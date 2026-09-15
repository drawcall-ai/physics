import { Object3D, Matrix4, Vector3 } from "three";
import { JointMotor, getJointMotor } from "./motor.js";
import { RigidBody } from "./body.js";
import { splitTransform } from "./transforms.js";
import { assertRigidTransform } from "./objects.js";
import type {
  PhysicsWorld,
  PhysicsJointState,
  AxisJointState,
} from "./world.js";

export type JointOptions = {
  readonly body0: RigidBody | null;
  readonly body1: RigidBody;
} & (
  | { readonly frame0?: never; readonly frame1?: never }
  | { readonly frame0: Matrix4; readonly frame1: Matrix4 }
);
export abstract class Joint<
  Options extends JointOptions = JointOptions,
> extends Object3D {
  readonly world: PhysicsWorld;
  protected readonly config: Options;
  get options(): Options {
    return {
      ...this.config,
      frame0: this.config.frame0?.clone(),
      frame1: this.config.frame1?.clone(),
    };
  }
  private isDisposed = false;
  private currentEnabled = true;
  private currentCollideConnected = false;
  private version = 0;
  get enabled(): boolean {
    return this.currentEnabled;
  }
  get collideConnected(): boolean {
    return this.currentCollideConnected;
  }
  get settingsVersion(): number {
    return this.version;
  }
  protected assertLive(): void {
    if (this.disposed) throw new Error("Joint has been disposed");
  }
  setEnabled(value: boolean): this {
    this.assertLive();
    this.currentEnabled = value;
    this.version++;
    if (!value && this instanceof AxisJoint) this.world.setJointEffort(this, 0);
    return this;
  }
  setCollideConnected(value: boolean): this {
    this.assertLive();
    this.currentCollideConnected = value;
    this.version++;
    return this;
  }

  constructor(options: Options) {
    super();
    this.world = options.body1.world;
    this.config = {
      ...options,
      frame0: options.frame0?.clone(),
      frame1: options.frame1?.clone(),
    };
    if (options.frame0) {
      assertRigidTransform(options.frame0);
      assertRigidTransform(options.frame1);
    }
    if (options.body0 && options.body0.world !== this.world)
      throw new Error("Joint bodies must belong to the same world");
    if (options.body0 === options.body1)
      throw new Error("Joint must connect distinct bodies");
    if (options.body0?.disposed || options.body1.disposed)
      throw new Error("Joint cannot connect disposed bodies");
    this.world.register(this);
  }

  get disposed(): boolean {
    return this.isDisposed;
  }

  dispose(): void {
    if (this.isDisposed) return;
    if (this instanceof AxisJoint) this.motor?.dispose();
    this.world.unregister(this);
    this.removeFromParent();
    this.isDisposed = true;
  }

  getState(): PhysicsJointState | AxisJointState {
    return connectionState(this);
  }

  override clone(recursive = true): this {
    return this.cloneWithBodies(new Map(), recursive);
  }

  cloneWithBodies(
    objects: ReadonlyMap<Object3D, Object3D>,
    recursive = true,
  ): this {
    if (this.disposed) throw new Error("Cannot clone a disposed joint");
    const target: unknown = Reflect.construct(this.constructor, [
      {
        ...this.config,
        body0: this.config.body0
          ? mappedBody(this.config.body0, objects)
          : null,
        body1: mappedBody(this.config.body1, objects),
      },
    ]);
    if (!this.isClone(target))
      throw new Error(
        "Joint clone constructor returned an incompatible object",
      );
    try {
      return objects.size
        ? target.copyState(this, recursive)
        : target.copy(this, recursive);
    } catch (error) {
      const created: (RigidBody | Joint)[] = [];
      target.traverse((object) => {
        if (object instanceof RigidBody || object instanceof Joint)
          created.push(object);
      });
      for (const object of created.reverse()) object.dispose();
      throw error;
    }
  }

  private isClone(value: unknown): value is this {
    return (
      value instanceof Joint &&
      Object.getPrototypeOf(value) === Object.getPrototypeOf(this)
    );
  }

  override copy(source: this, recursive = true): this {
    if (source === this) return this;
    if (this.disposed || source.disposed)
      throw new Error("Cannot copy a disposed joint");
    const a = this.config,
      b = source.config;
    if (
      a.body0 !== b.body0 ||
      a.body1 !== b.body1 ||
      !sameFrame(a.frame0, b.frame0) ||
      !sameFrame(a.frame1, b.frame1) ||
      (this instanceof AxisJoint &&
        source instanceof AxisJoint &&
        ((this.config.axis ?? "Y") !== (source.config.axis ?? "Y") ||
          !sameLimits(this.limits, source.limits))) ||
      (this instanceof DistanceJoint &&
        source instanceof DistanceJoint &&
        !sameLimits(this.limits, source.limits))
    )
      throw new Error("Joint copy requires matching immutable options");
    return this.copyState(source, recursive);
  }

  private copyState(source: this, recursive: boolean): this {
    super.copy(source, recursive);
    this.setEnabled(source.enabled).setCollideConnected(
      source.collideConnected,
    );
    if (this instanceof AxisJoint && source instanceof AxisJoint) {
      this.motor?.dispose();
      if (source.motor) {
        const motor = new JointMotor({ ...source.motor.options, joint: this });
        if (source.motor.target) motor.setTarget(source.motor.target);
        motor.setEnabled(source.motor.enabled);
      }
    }
    return this;
  }

  validate(): void {
    if (
      this.disposed ||
      this.config.body0?.disposed ||
      this.config.body1.disposed
    )
      throw new Error("Cannot validate a disposed joint or body");
    this.config.body0?.updateWorldMatrix(true, false);
    this.config.body1.updateWorldMatrix(true, false);
    this.updateWorldMatrix(true, false);
    splitTransform(this.matrixWorld);
    if (this.config.body0) splitTransform(this.config.body0.matrixWorld);
    splitTransform(this.config.body1.matrixWorld);
  }

  getFrame(index: 0 | 1, target: Matrix4): Matrix4 {
    this.validate();
    const frame = index === 0 ? this.config.frame0 : this.config.frame1;
    const body = index === 0 ? this.config.body0 : this.config.body1;
    if (frame) {
      target.copy(frame);
      if (body)
        target.setPosition(
          new Vector3()
            .setFromMatrixPosition(frame)
            .multiply(splitTransform(body.matrixWorld).scale),
        );
      return target;
    }
    target.copy(splitTransform(this.matrixWorld).pose);
    return body
      ? target.premultiply(splitTransform(body.matrixWorld).pose.invert())
      : target;
  }
}
export class FixedJoint extends Joint {
  override getState(): PhysicsJointState {
    return connectionState(this);
  }
}
export type AxisJointOptions = JointOptions & {
  readonly axis?: "X" | "Y" | "Z";
  readonly limits?: readonly [number, number];
};
export abstract class AxisJoint extends Joint<AxisJointOptions> {
  constructor(options: AxisJointOptions) {
    if (options.limits) validateLimits(options.limits);
    super({
      ...options,
      limits: options.limits && [...options.limits],
    });
  }
  get limits(): readonly [number, number] | undefined {
    return this.config.limits;
  }
  get motor(): JointMotor | undefined {
    return getJointMotor(this);
  }
  override getState(): AxisJointState {
    const state = this.world.getJointState(this);
    return this instanceof RevoluteJoint
      ? { position: state.angle, velocity: state.angularVelocity }
      : { position: state.position, velocity: state.velocity };
  }
  setEffort(value: number): this {
    this.assertLive();
    if (!Number.isFinite(value)) throw new Error("Joint effort must be finite");
    if (value !== 0 && this.motor?.active)
      throw new Error("Disable the joint motor before applying effort");
    this.world.setJointEffort(this, value);
    return this;
  }
}
export class RevoluteJoint extends AxisJoint {}
export class PrismaticJoint extends AxisJoint {}
export class SphericalJoint extends Joint {
  override getState(): PhysicsJointState {
    return connectionState(this);
  }
}
export type DistanceJointOptions = JointOptions & {
  readonly limits?: readonly [number, number];
};
export class DistanceJoint extends Joint<
  DistanceJointOptions & { readonly limits: readonly [number, number] }
> {
  constructor(options: DistanceJointOptions) {
    const limits = options.limits ?? [0, 0];
    validateLimits(limits);
    if (limits[0] < 0) throw new Error("Distance limits must be nonnegative");
    super({
      ...options,
      limits: [limits[0], limits[1]],
    });
  }
  override getState(): PhysicsJointState {
    return connectionState(this);
  }
  get limits(): readonly [number, number] {
    return this.config.limits;
  }
}
function validateLimits(value: readonly [number, number]): void {
  if (![value[0], value[1]].every(Number.isFinite) || value[0] > value[1])
    throw new Error("Invalid joint limits");
}
function sameFrame(a: Matrix4 | undefined, b: Matrix4 | undefined): boolean {
  return a === undefined ? b === undefined : b !== undefined && a.equals(b);
}

function mappedBody(
  body: RigidBody,
  objects: ReadonlyMap<Object3D, Object3D>,
): RigidBody {
  const target = objects.get(body) ?? body;
  if (
    !(target instanceof RigidBody) ||
    target.disposed ||
    target.world !== body.world
  )
    throw new Error("Joint copy requires live bodies in the same world");
  return target;
}

function sameLimits(
  a: readonly [number, number] | undefined,
  b: readonly [number, number] | undefined,
): boolean {
  return a === undefined
    ? b === undefined
    : b !== undefined && a[0] === b[0] && a[1] === b[1];
}

function connectionState(joint: Joint): PhysicsJointState {
  const { velocity: _velocity, ...state } = joint.world.getJointState(joint);
  return state;
}
