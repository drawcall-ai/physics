import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  Trigger,
  AxisJoint,
  GenericJoint,
  SphericalJoint,
  splitTransform,
  resolveCollider,
  authoredVelocity,
  authoredJointReading,
  type JointReading,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import {
  compile,
  type Compiled,
  type BodyRecord,
  type ModelOptions,
} from "./model.js";
import type { JointRecord } from "./joints.js";
import { array, at, name } from "./values.js";
import { writeVelocity } from "./motion.js";

/** Owns captured authoring state and atomically replaces the compiled model after scene edits. */
export class Scene {
  readonly objects = new Set<RigidBody>();
  readonly bodies = new Map<RigidBody, BodyRecord>();
  readonly jointObjects = new Set<Joint>();
  readonly joints = new Map<Joint, JointRecord>();
  readonly triggers = new Set<Trigger>();
  readonly scales = new Map<Object3D, Vector3>();
  compiled?: Compiled;
  key = "";
  constructor(
    private readonly api: MainModule,
    private readonly options: ModelOptions,
  ) {}
  prepare(time: number, read: (joint: Joint) => JointReading): Compiled {
    for (const body of this.objects) {
      body.validate();
      if (!this.bodies.has(body))
        this.bodies.set(body, {
          initialPose: splitTransform(body.matrixWorld).pose,
          initialVelocity: authoredVelocity(body),
        });
    }
    for (const joint of this.jointObjects) {
      joint.validate();
      if (!this.joints.has(joint)) {
        const frames: [Matrix4, Matrix4] = [
          joint.getFrame(0, new Matrix4()),
          joint.getFrame(1, new Matrix4()),
        ];
        const angle = authoredJointReading(
          joint,
          frames,
          () => new Vector3(),
        ).angle;
        this.joints.set(joint, { frames, angle, sampled: angle });
      }
    }
    const key = fingerprint(
      this.objects,
      this.jointObjects,
      this.triggers,
      this.scales,
    );
    if (this.compiled && key === this.key) return this.compiled;
    const velocities = new Map(
      [...this.objects].map((body) => [body, body.getVelocity()]),
    );
    const previous = this.compiled;
    const speeds = new Map<string, number[]>();
    if (previous)
      for (let j = 0; j < previous.model.njnt; j++) {
        const key = this.api.mj_id2name(
          previous.model,
          this.api.mjtObj.mjOBJ_JOINT.value,
          j,
        );
        const start = at(previous.model.jnt_dofadr, j),
          type = at(previous.model.jnt_type, j);
        speeds.set(
          key,
          Array.from(
            array(previous.data.qvel).slice(
              start,
              start + (type === 0 ? 6 : type === 1 ? 3 : 1),
            ),
          ),
        );
      }
    const next = compile(
      this.api,
      this.bodies,
      this.joints,
      this.triggers,
      this.options,
      (joint) => read(joint),
    );
    try {
      for (const [body, id] of next.bodies) {
        const value = velocities.get(body);
        if (body.bodyType === "dynamic" && next.roots.has(body) && value)
          writeVelocity(next, id, value);
      }
      for (let j = 0; j < next.model.njnt; j++) {
        if (at(next.model.jnt_type, j) === 0) continue;
        const key = this.api.mj_id2name(
          next.model,
          this.api.mjtObj.mjOBJ_JOINT.value,
          j,
        );
        const values = speeds.get(key);
        const address = at(next.model.jnt_dofadr, j);
        if (values) {
          array(next.data.qvel).set(values, address);
          continue;
        }
        const coordinate = next.coordinates.get(j);
        if (coordinate?.joint instanceof AxisJoint) {
          array(next.data.qvel)[address] = coordinate.joint.getState().velocity;
        } else if (
          coordinate?.joint instanceof GenericJoint &&
          coordinate.axis
        ) {
          array(next.data.qvel)[address] = coordinate.joint.getState(
            coordinate.axis,
          ).velocity;
        } else {
          const joint = [...this.jointObjects].find(
            (joint) => name(joint) === key,
          );
          if (joint instanceof SphericalJoint) {
            const { body0, body1 } = joint.options;
            const angular = body1
              .getVelocity()
              .angular.sub(body0?.getVelocity().angular ?? new Vector3());
            angular.applyQuaternion(
              new Quaternion()
                .setFromRotationMatrix(splitTransform(body1.matrixWorld).pose)
                .invert(),
            );
            array(next.data.qvel).set(angular.toArray(), address);
          }
        }
      }
      next.data.time = time;
      this.api.mj_forward(next.model, next.data);
    } catch (error) {
      next.free();
      throw error;
    }
    this.compiled = next;
    this.key = key;
    previous?.free();
    return next;
  }
  preview(time: number, read: (joint: Joint) => JointReading): Compiled {
    const scene = new Scene(this.api, this.options);
    for (const body of this.objects) scene.objects.add(body);
    for (const joint of this.jointObjects) scene.jointObjects.add(joint);
    for (const [joint, record] of this.joints) scene.joints.set(joint, record);
    for (const trigger of this.triggers) scene.triggers.add(trigger);
    return scene.prepare(time, read);
  }
  free(): void {
    this.compiled?.free();
    this.compiled = undefined;
    this.key = "";
  }
}
/** Captured scale is immutable; other authored edits trigger a transactional model rebuild. */
export function fingerprint(
  bodies: Iterable<RigidBody>,
  joints: Iterable<Joint>,
  triggers: Iterable<Trigger>,
  scales: Map<Object3D, Vector3>,
): string {
  const owners = [...bodies, ...triggers];
  return JSON.stringify(
    [
      owners.map((owner) => {
        const colliders = owner.getColliders();
        const bodyScale = splitTransform(owner.matrixWorld).scale;
        capture(owner, bodyScale, scales);
        return [
          owner.id,
          owner.settingsVersion,
          owner instanceof RigidBody ? owner.materialVersion : 0,
          colliders.map((collider) => {
            const part = resolveCollider(owner, collider);
            capture(collider.source, part.scale, scales);
            const shape = part.shape;
            const geometry =
              shape.kind === "mesh"
                ? [
                    shape.approximation,
                    Array.from(shape.geometry.getAttribute("position").array),
                    shape.geometry.index
                      ? Array.from(shape.geometry.index.array)
                      : null,
                  ]
                : shape;
            if (shape.kind === "mesh") shape.geometry.dispose();
            return [
              collider.source.id,
              collider.settingsVersion,
              geometry,
              part.matrix.elements.map((v) => Math.round(v * 1e8) / 1e8),
            ];
          }),
        ];
      }),
      [...joints].map((joint) => [
        joint.id,
        joint.enabled,
        joint.collideConnected,
        joint.settingsVersion,
      ]),
    ],
    (_key, value: unknown) =>
      typeof value === "number" ? Math.round(value * 1e8) / 1e8 : value,
  );
}
function capture(
  object: Object3D,
  scale: Vector3,
  scales: Map<Object3D, Vector3>,
): void {
  const captured = scales.get(object);
  if (captured && captured.distanceTo(scale) > 1e-6)
    throw new Error(
      `Physics scale cannot change after backend initialization: ${object.name || object.type}; recreate the body`,
    );
  if (!captured) scales.set(object, scale);
}
