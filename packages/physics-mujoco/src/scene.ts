import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  Trigger,
  splitTransform,
  authoredVelocity,
  authoredJointReading,
  type JointReading,
  type PhysicsVelocity,
  setWorldPose,
  setAuthoredVelocity,
  wrapAngle,
  cleanup,
  rollback,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { compile, type Compiled, type ModelOptions } from "./model/compile.js";
import type { JointRecord } from "./model/joints.js";
import { captureState } from "./model/state.js";
import { Changes } from "./changes.js";

interface BodyRecord {
  initialPose: Matrix4;
  initialVelocity: PhysicsVelocity;
}

/** Owns captured authoring state and atomically replaces the compiled model after scene edits. */
export class Scene {
  private readonly objects = new Set<RigidBody>();
  private readonly bodies = new Map<RigidBody, BodyRecord>();
  private readonly jointObjects = new Set<Joint>();
  private readonly records = new Map<Joint, JointRecord>();
  private readonly triggers = new Set<Trigger>();
  private readonly changes = new Changes();
  private current?: Compiled;
  private key = "";
  get compiled(): Compiled | undefined {
    return this.current;
  }
  get joints(): ReadonlyMap<Joint, JointRecord> {
    return this.records;
  }
  constructor(
    private readonly api: MainModule,
    private readonly options: ModelOptions,
  ) {}
  register(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody) this.objects.add(object);
    else if (object instanceof Joint) this.jointObjects.add(object);
    else this.triggers.add(object);
  }
  unregister(object: RigidBody | Joint | Trigger): void {
    this.key = "";
    if (object instanceof Joint) {
      this.jointObjects.delete(object);
      this.records.delete(object);
      return;
    }
    this.changes.remove(object);
    if (object instanceof Trigger) {
      this.triggers.delete(object);
      return;
    }
    this.objects.delete(object);
    this.bodies.delete(object);
  }
  trackAngles(): void {
    for (const [joint, record] of this.records) {
      const sampled = authoredJointReading(
        joint,
        record.frames,
        () => new Vector3(),
      ).angle;
      record.angle += wrapAngle(sampled - record.sampled);
      record.sampled = sampled;
    }
  }
  reset(): void {
    this.free();
    for (const [body, record] of this.bodies) {
      setWorldPose(body, record.initialPose);
      setAuthoredVelocity(body, record.initialVelocity);
    }
    for (const [joint, record] of this.records) {
      record.angle = authoredJointReading(
        joint,
        record.frames,
        () => new Vector3(),
      ).angle;
      record.sampled = record.angle;
    }
  }
  dispose(): void {
    cleanup(
      [...this.triggers, ...this.jointObjects, ...this.objects]
        .map((object) => () => object.dispose())
        .concat(() => this.changes.clear()),
      "MuJoCo scene disposal failed",
    );
  }
  prepare(time: number, read: (joint: Joint) => JointReading): Compiled {
    const bodies = new Map(this.bodies);
    const joints = new Map(this.records);
    for (const body of this.objects) {
      body.validate();
      if (!bodies.has(body))
        bodies.set(body, {
          initialPose: splitTransform(body.matrixWorld).pose,
          initialVelocity: authoredVelocity(body),
        });
    }
    for (const joint of this.jointObjects) {
      joint.validate();
      if (!joints.has(joint)) {
        const frames: [Matrix4, Matrix4] = [
          joint.getFrame(0, new Matrix4()),
          joint.getFrame(1, new Matrix4()),
        ];
        const angle = authoredJointReading(
          joint,
          frames,
          () => new Vector3(),
        ).angle;
        joints.set(joint, { frames, angle, sampled: angle });
      }
    }
    const change = this.changes.scan(
      this.objects,
      this.jointObjects,
      this.triggers,
    );
    const key = change.key;
    if (this.compiled && key === this.key) return this.compiled;
    const previous = this.compiled;
    const restoreState = captureState(
      this.api,
      previous,
      this.objects,
      this.jointObjects,
    );
    const next = compile(
      this.api,
      this.objects,
      joints,
      this.triggers,
      this.options,
      (joint) => read(joint),
    );
    try {
      restoreState(next);
      next.data.time = time;
      this.api.mj_forward(next.model, next.data);
    } catch (error) {
      rollback(error, [() => next.free()], "MuJoCo model replacement failed");
    }
    for (const [body, record] of bodies) this.bodies.set(body, record);
    for (const [joint, record] of joints) this.records.set(joint, record);
    this.current = next;
    this.key = key;
    this.changes.commit(change.scales);
    previous?.free();
    return next;
  }
  /** The live model when it still matches the authored scene; queries reuse it instead of compiling. */
  matching(): Compiled | undefined {
    if (!this.current) return undefined;
    const change = this.changes.scan(
      this.objects,
      this.jointObjects,
      this.triggers,
    );
    return change.key === this.key ? this.current : undefined;
  }
  /** Compiles the authored scene without committing it, so queries leave the live scene editable. */
  preview(time: number, read: (joint: Joint) => JointReading): Compiled {
    const scene = new Scene(this.api, this.options);
    for (const body of this.objects) scene.objects.add(body);
    for (const joint of this.jointObjects) scene.jointObjects.add(joint);
    for (const [joint, record] of this.joints) scene.records.set(joint, record);
    for (const trigger of this.triggers) scene.triggers.add(trigger);
    return scene.prepare(time, read);
  }
  previewBody(body: RigidBody): Compiled {
    body.validate();
    return compile(
      this.api,
      new Set([body]),
      new Map(),
      new Set(),
      this.options,
      () => {
        throw new Error("A body preview cannot contain joints");
      },
    );
  }
  free(): void {
    this.compiled?.free();
    this.current = undefined;
    this.key = "";
  }
}
