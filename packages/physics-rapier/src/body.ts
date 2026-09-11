import type {
  RigidBody as AuthoredBody,
  PhysicsBodyControls,
  Collider,
} from "@drawcall/physics";
import type { RigidBody } from "@dimforge/rapier3d-compat";
import type * as Rapier from "@dimforge/rapier3d-compat";
import { collider } from "./shapes.js";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import {
  assertRigidTransform,
  splitTransform,
  resolveCollider,
} from "@drawcall/physics";

export class BodyControls implements PhysicsBodyControls {
  constructor(
    private readonly resolve: () => RigidBody,
    private readonly object: AuthoredBody,
  ) {}
  getMatrix(target = new Matrix4()): Matrix4 {
    const body = this.resolve();
    return target.compose(
      new Vector3().copy(body.translation()),
      new Quaternion().copy(body.rotation()).normalize(),
      new Vector3(1, 1, 1),
    );
  }
  getVelocity() {
    const body = this.resolve();
    return {
      linear: new Vector3().copy(body.linvel()),
      angular: new Vector3().copy(body.angvel()),
    };
  }
  setVelocity(value: { linear?: Vector3; angular?: Vector3 }): void {
    const body = this.resolve();
    if (value.linear) body.setLinvel(value.linear, true);
    if (value.angular) body.setAngvel(value.angular, true);
  }
  setKinematicTarget(matrix: Matrix4): void {
    if (this.object.options.type !== "kinematic")
      throw new Error("Kinematic targets require a kinematic body.");
    assertRigidTransform(matrix);
    const body = this.resolve();
    body.setNextKinematicTranslation(
      new Vector3().setFromMatrixPosition(matrix),
    );
    body.setNextKinematicRotation(
      new Quaternion().setFromRotationMatrix(matrix),
    );
  }
  teleport(matrix: Matrix4): void {
    assertRigidTransform(matrix);
    const body = this.resolve();
    body.setTranslation(new Vector3().setFromMatrixPosition(matrix), true);
    body.setRotation(new Quaternion().setFromRotationMatrix(matrix), true);
    synchronize(this.object, body);
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    if (point) this.resolve().applyImpulseAtPoint(impulse, point, true);
    else this.resolve().applyImpulse(impulse, true);
  }
  applyForce(force: Vector3, point?: Vector3): void {
    if (point) this.resolve().addForceAtPoint(force, point, true);
    else this.resolve().addForce(force, true);
  }
  wake(): void {
    this.resolve().wakeUp();
  }
  sleep(): void {
    this.resolve().sleep();
  }
}

export function synchronize(object: AuthoredBody, body: RigidBody): void {
  if (!object.matrixAutoUpdate)
    object.scale.copy(splitTransform(object.matrix).scale);
  const p = body.translation(),
    q = body.rotation();
  object.position.set(p.x, p.y, p.z);
  object.quaternion.set(q.x, q.y, q.z, q.w).normalize();
  if (object.parent) {
    object.parent.updateWorldMatrix(true, false);
    object.parent.worldToLocal(object.position);
    object.quaternion.premultiply(
      object.parent.getWorldQuaternion(new Quaternion()).invert(),
    );
  }
  object.updateMatrix();
  object.updateMatrixWorld(true);
}

export interface BodyBinding {
  body: Rapier.RigidBody;
  initial: Matrix4;
  shapes: string;
  settings: string;
  canSleep: boolean;
  scale: Vector3;
  colliderScales: Map<Object3D, Vector3>;
}

export function createBody(
  api: typeof Rapier,
  world: Rapier.World,
  object: AuthoredBody,
): BodyBinding {
  const options = object.options;
  const desc =
    options.type === "static"
      ? api.RigidBodyDesc.fixed()
      : options.type === "kinematic"
        ? api.RigidBodyDesc.kinematicPositionBased()
        : api.RigidBodyDesc.dynamic();
  const { pose, scale } = splitTransform(object.matrixWorld);
  const position = new Vector3().setFromMatrixPosition(pose);
  desc
    .setTranslation(position.x, position.y, position.z)
    .setRotation(new Quaternion().setFromRotationMatrix(pose))
    .setCanSleep(options.canSleep ?? true)
    .setLinvel(...(options.linearVelocity ?? [0, 0, 0]))
    .setAngvel(new Vector3(...(options.angularVelocity ?? [0, 0, 0])));
  const body = world.createRigidBody(desc);
  const binding: BodyBinding = {
    body,
    initial: pose,
    scale,
    colliderScales: new Map(),
    shapes: "",
    settings: "",
    canSleep: options.canSleep ?? true,
  };
  try {
    refreshBody(api, world, object, binding);
    return binding;
  } catch (error) {
    world.removeRigidBody(body);
    throw error;
  }
}

export function refreshBody(
  api: typeof Rapier,
  world: Rapier.World,
  object: AuthoredBody,
  binding: BodyBinding,
): void {
  const options = object.options;
  if (binding.canSleep !== (options.canSleep ?? true)) {
    throw new Error("canSleep cannot change after a body is created.");
  }
  if (splitTransform(object.matrixWorld).scale.distanceTo(binding.scale) > 1e-6)
    throw new Error(
      "Body scale cannot change after its first physics step; recreate the body",
    );
  const colliders = object.getColliders();
  const shapes = JSON.stringify([
    options.mass,
    colliders.map((shape) => shapeKey(shape, object)),
  ]);
  const body = binding.body;
  if (shapes !== binding.shapes) {
    const resolved = colliders.map((collider) =>
      resolveCollider(
        object,
        collider,
        binding.colliderScales.get(collider.source),
      ),
    );
    for (const { collider, scale } of resolved) {
      const captured = binding.colliderScales.get(collider.source);
      if (captured && captured.distanceTo(scale) > 1e-6)
        throw new Error(
          `Collider scale cannot change after its first physics step: ${object.name}/${collider.name || collider.type} (${captured.toArray()} → ${scale.toArray()}); recreate the body`,
        );
    }
    const descriptors = resolved.map((shape) => collider(api, shape, object));
    const next: Rapier.Collider[] = [];
    try {
      for (const desc of descriptors)
        next.push(world.createCollider(desc, body));
      if (options.mass !== undefined) {
        const volume = next.reduce((sum, shape) => sum + shape.volume(), 0);
        if (volume <= 0)
          throw new Error(
            "Explicit mass requires colliders with positive volume.",
          );
        for (const shape of next)
          shape.setMass((options.mass * shape.volume()) / volume);
      }
    } catch (error) {
      for (const shape of next) world.removeCollider(shape, true);
      throw error;
    }
    const previous = body.numColliders() - next.length;
    for (let i = 0; i < previous; i++)
      world.removeCollider(body.collider(0), true);
    body.recomputeMassPropertiesFromColliders();
    body.wakeUp();
    binding.shapes = shapes;
    binding.colliderScales = new Map(
      resolved.map(({ collider, scale }) => [
        collider.source,
        binding.colliderScales.get(collider.source) ?? scale,
      ]),
    );
  }
  const settings = JSON.stringify([
    options.type,
    options.linearDamping,
    options.angularDamping,
    options.gravityScale,
    options.canSleep,
  ]);
  if (settings === binding.settings) return;
  const type =
    options.type === "static"
      ? api.RigidBodyType.Fixed
      : options.type === "kinematic"
        ? api.RigidBodyType.KinematicPositionBased
        : api.RigidBodyType.Dynamic;
  body.setBodyType(type, true);
  body.setLinearDamping(options.linearDamping ?? 0);
  body.setAngularDamping(options.angularDamping ?? 0);
  body.setGravityScale(options.gravityScale ?? 1, true);
  binding.settings = settings;
}

function shapeKey(source: Collider, body: AuthoredBody): unknown {
  const shape = source.shape();
  const { sensor, collisionGroups } = source;
  const material = body.getMaterial(source);
  const shapeData =
    shape.kind === "mesh"
      ? {
          kind: shape.kind,
          approximation: shape.approximation,
          positions: Array.from(shape.geometry.getAttribute("position").array),
          indices: shape.geometry.index
            ? Array.from(shape.geometry.index.array)
            : null,
        }
      : shape;
  // Relative transforms accumulate tiny roundoff as bodies move under parents.
  const transform = body.matrixWorld
    .clone()
    .invert()
    .multiply(source.matrixWorld)
    .elements.map((value) => Math.round(value * 1e10) / 1e10);
  return [
    source.source.uuid,
    shapeData,
    transform,
    material,
    sensor,
    collisionGroups,
  ];
}
