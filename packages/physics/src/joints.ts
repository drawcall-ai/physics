import { Object3D, Matrix4 } from "three";
import { RigidBody } from "./body.js";
import { assertRigidTransform } from "./objects.js";
import type { PhysicsWorld } from "./world.js";

export interface JointDrive {
  type?: "force" | "acceleration";
  targetPosition?: number;
  targetVelocity?: number;
  stiffness?: number;
  damping?: number;
  maxForce?: number;
}
export interface JointOptions {
  readonly body0: RigidBody | null;
  readonly body1: RigidBody;
  frame0?: Matrix4;
  frame1?: Matrix4;
  collideConnected?: boolean;
  enabled?: boolean;
}
export class Joint<
  Options extends JointOptions = JointOptions,
> extends Object3D {
  readonly world: PhysicsWorld;
  #disposed = false;

  constructor(readonly options: Options) {
    super();
    this.world = options.body1.world;
    if (options.body0 && options.body0.world !== this.world)
      throw new Error("Joint bodies must belong to the same world");
    if (options.body0 === options.body1)
      throw new Error("Joint must connect distinct bodies");
    if (options.body0?.disposed || options.body1.disposed)
      throw new Error("Joint cannot connect disposed bodies");
    this.world.register(this);
  }

  get disposed(): boolean {
    return this.#disposed;
  }

  dispose(): void {
    if (this.#disposed) return;
    this.world.unregister(this);
    this.removeFromParent();
    this.#disposed = true;
  }

  override clone(recursive = true): this {
    if (this.disposed) throw new Error("Cannot clone a disposed joint");
    const target: unknown = Reflect.construct(this.constructor, [
      { ...this.options },
    ]);
    if (!this.isClone(target))
      throw new Error(
        "Joint clone constructor returned an incompatible object",
      );
    try {
      return target.copy(this, recursive);
    } catch (error) {
      const objects: (RigidBody | Joint)[] = [];
      target.traverse((object) => {
        if (object instanceof RigidBody || object instanceof Joint)
          objects.push(object);
      });
      for (const object of objects.reverse()) object.dispose();
      throw error;
    }
  }

  private isClone(value: unknown): value is this {
    return (
      value instanceof Joint &&
      Object.getPrototypeOf(value) === Object.getPrototypeOf(this)
    );
  }

  override copy(
    source: this,
    recursive = true,
    objects: ReadonlyMap<Object3D, Object3D> = new Map(),
  ): this {
    if (this.disposed || source.disposed)
      throw new Error("Cannot copy a disposed joint");
    const body = (original: RigidBody) => {
      const target = objects.get(original) ?? original;
      if (
        !(target instanceof RigidBody) ||
        target.world !== this.world ||
        target.disposed
      )
        throw new Error("Joint copy requires live bodies in the same world");
      return target;
    };
    const options = {
      ...source.options,
      body0: source.options.body0 ? body(source.options.body0) : null,
      body1: body(source.options.body1),
      frame0: source.options.frame0?.clone(),
      frame1: source.options.frame1?.clone(),
    };
    super.copy(source, recursive);
    for (const key of Object.keys(this.options))
      Reflect.deleteProperty(this.options, key);
    Object.assign(this.options, options);
    if (this instanceof AxisJoint && source instanceof AxisJoint) {
      this.options.limits = source.options.limits
        ? [...source.options.limits]
        : undefined;
      this.options.drive = source.options.drive
        ? { ...source.options.drive }
        : undefined;
    }
    if (this instanceof DistanceJoint && source instanceof DistanceJoint)
      this.options.limits = [...source.options.limits];
    return this;
  }

  validate(): void {
    if (
      this.disposed ||
      this.options.body0?.disposed ||
      this.options.body1.disposed
    )
      throw new Error("Cannot validate a disposed joint or body");
    if (
      this.options.body1.world !== this.world ||
      (this.options.body0 && this.options.body0.world !== this.world)
    )
      throw new Error("Joint bodies must belong to the same world");
    this.options.body0?.updateWorldMatrix(true, false);
    this.options.body1.updateWorldMatrix(true, false);
    this.updateWorldMatrix(true, false);
    if (this.options.body0 === this.options.body1)
      throw new Error("Joint must connect distinct bodies");
    if (
      (this.options.frame0 === undefined) !==
      (this.options.frame1 === undefined)
    )
      throw new Error("Joint requires both local frames or neither");
    if (this instanceof AxisJoint || this instanceof DistanceJoint) {
      if (
        this.options.limits &&
        (!this.options.limits.every(Number.isFinite) ||
          this.options.limits[0] > this.options.limits[1])
      )
        throw new Error("Invalid joint limits");
    }
    if (this instanceof DistanceJoint && this.options.limits[0] < 0)
      throw new Error("Distance limits must be nonnegative");
    if (this instanceof AxisJoint && this.options.drive)
      validateDrive(this.options.drive);
    assertRigidTransform(this.matrixWorld);
    if (this.options.body0)
      assertRigidTransform(this.options.body0.matrixWorld);
    assertRigidTransform(this.options.body1.matrixWorld);
    if (this.options.frame0) assertRigidTransform(this.options.frame0);
    if (this.options.frame1) assertRigidTransform(this.options.frame1);
  }

  getFrame(index: 0 | 1, target: Matrix4): Matrix4 {
    this.validate();
    const frame = index === 0 ? this.options.frame0 : this.options.frame1;
    if (frame) return target.copy(frame);
    const body = index === 0 ? this.options.body0 : this.options.body1;
    return body
      ? target.copy(body.matrixWorld).invert().multiply(this.matrixWorld)
      : target.copy(this.matrixWorld);
  }
}
export class FixedJoint extends Joint {}
export interface AxisJointOptions extends JointOptions {
  axis?: "X" | "Y" | "Z";
  limits?: [number, number];
  drive?: JointDrive;
}
export class AxisJoint extends Joint<AxisJointOptions> {}
export class RevoluteJoint extends AxisJoint {}
export class PrismaticJoint extends AxisJoint {}
export class SphericalJoint extends Joint {}
export interface DistanceJointOptions extends JointOptions {
  limits: [number, number];
}
export class DistanceJoint extends Joint<DistanceJointOptions> {}

export function validateDrive(drive: JointDrive): void {
  if (
    ![drive.targetPosition ?? 0, drive.targetVelocity ?? 0].every(
      Number.isFinite,
    )
  ) {
    throw new Error("Joint drive targets must be finite");
  }
  if (
    ![drive.stiffness ?? 0, drive.damping ?? 0, drive.maxForce ?? 0].every(
      (value) => Number.isFinite(value) && value >= 0,
    )
  ) {
    throw new Error(
      "Joint drive coefficients and maximum force must be finite and nonnegative",
    );
  }
}
