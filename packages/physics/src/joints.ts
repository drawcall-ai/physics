import { Object3D, Matrix4, Vector3 } from "three";
import { JointMotor, getJointMotor } from "./motor.js";
import { RigidBody } from "./body.js";
import { splitTransform } from "./transforms.js";
import { assertRigidTransform } from "./objects.js";
import type { PhysicsWorld, PhysicsJointState } from "./world.js";

export interface JointOptions {
  readonly body0: RigidBody | null;
  readonly body1: RigidBody;
  readonly frame0?: Matrix4;
  readonly frame1?: Matrix4;
}
export abstract class Joint<
  Options extends JointOptions = JointOptions,
> extends Object3D {
  #world: PhysicsWorld;
  get world(): PhysicsWorld {
    return this.#world;
  }
  #disposed = false;
  #enabled = true;
  #collideConnected = false;
  #version = 0;
  #options: Options;
  get options(): Options {
    return Object.freeze({
      ...this.#options,
      frame0: this.#options.frame0?.clone(),
      frame1: this.#options.frame1?.clone(),
    });
  }
  get enabled(): boolean {
    return this.#enabled;
  }
  get collideConnected(): boolean {
    return this.#collideConnected;
  }
  get settingsVersion(): number {
    return this.#version;
  }
  protected changed(): void {
    this.#version++;
  }
  protected assertLive(): void {
    if (this.disposed) throw new Error("Joint has been disposed");
  }
  setEnabled(value: boolean): this {
    this.assertLive();
    this.#enabled = value;
    this.changed();
    if (!value && this instanceof AxisJoint) this.world.setJointEffort(this, 0);
    return this;
  }
  setCollideConnected(value: boolean): this {
    this.assertLive();
    this.#collideConnected = value;
    this.changed();
    return this;
  }

  constructor(options: Options) {
    super();
    this.#options = Object.freeze({
      ...options,
      frame0: options.frame0?.clone(),
      frame1: options.frame1?.clone(),
    });
    if ((options.frame0 === undefined) !== (options.frame1 === undefined))
      throw new Error("Joint requires both local frames or neither");
    if (options.frame0) assertRigidTransform(options.frame0);
    if (options.frame1) assertRigidTransform(options.frame1);
    this.#world = options.body1.world;
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
    if (this instanceof AxisJoint) this.motor?.dispose();
    this.world.unregister(this);
    this.removeFromParent();
    this.#disposed = true;
  }

  getState(): PhysicsJointState {
    return this.world.getJointState(this);
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
        ...this.options,
        body0: this.options.body0
          ? mappedBody(this.options.body0, objects)
          : null,
        body1: mappedBody(this.options.body1, objects),
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
    const a = this.#options,
      b = source.#options;
    if (
      a.body0 !== b.body0 ||
      a.body1 !== b.body1 ||
      !sameFrame(a.frame0, b.frame0) ||
      !sameFrame(a.frame1, b.frame1) ||
      (this instanceof AxisJoint &&
        source instanceof AxisJoint &&
        ((this.options.axis ?? "Y") !== (source.options.axis ?? "Y") ||
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
      this.#options.body0?.disposed ||
      this.#options.body1.disposed
    )
      throw new Error("Cannot validate a disposed joint or body");
    if (
      this.#options.body1.world !== this.world ||
      (this.#options.body0 && this.#options.body0.world !== this.world)
    )
      throw new Error("Joint bodies must belong to the same world");
    this.#options.body0?.updateWorldMatrix(true, false);
    this.#options.body1.updateWorldMatrix(true, false);
    this.updateWorldMatrix(true, false);
    if (this.#options.body0 === this.#options.body1)
      throw new Error("Joint must connect distinct bodies");
    if (
      (this.#options.frame0 === undefined) !==
      (this.#options.frame1 === undefined)
    )
      throw new Error("Joint requires both local frames or neither");
    splitTransform(this.matrixWorld);
    if (this.#options.body0) splitTransform(this.#options.body0.matrixWorld);
    splitTransform(this.#options.body1.matrixWorld);
    if (this.#options.frame0) assertRigidTransform(this.#options.frame0);
    if (this.#options.frame1) assertRigidTransform(this.#options.frame1);
  }

  getFrame(index: 0 | 1, target: Matrix4): Matrix4 {
    this.validate();
    const frame = index === 0 ? this.#options.frame0 : this.#options.frame1;
    const body = index === 0 ? this.#options.body0 : this.#options.body1;
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
  override getState(): import("./world.js").FixedJointState {
    const state = super.getState();
    if (!("translation" in state))
      throw new Error("Backend returned invalid fixed joint state");
    return state;
  }
}
export interface AxisJointOptions extends JointOptions {
  readonly axis?: "X" | "Y" | "Z";
  readonly limits?: readonly [number, number];
}
export abstract class AxisJoint extends Joint<AxisJointOptions> {
  constructor(options: AxisJointOptions) {
    if (options.limits) validateLimits(options.limits);
    super({
      ...options,
      limits:
        options.limits &&
        Object.freeze<readonly [number, number]>([...options.limits]),
    });
  }
  get limits(): readonly [number, number] | undefined {
    return this.options.limits;
  }
  get motor(): JointMotor | undefined {
    return getJointMotor(this);
  }
  setEffort(value: number): this {
    this.assertLive();
    if (!Number.isFinite(value)) throw new Error("Joint effort must be finite");
    if (value !== 0 && this.motor?.active)
      throw new Error("Disable the joint motor before applying effort");
    this.world.setJointEffort(this, value);
    return this;
  }
  override getState(): import("./world.js").AxisJointState {
    const state = super.getState();
    if (!("position" in state))
      throw new Error("Backend returned invalid axis joint state");
    return state;
  }
}
export class RevoluteJoint extends AxisJoint {}
export class PrismaticJoint extends AxisJoint {}
export class SphericalJoint extends Joint {
  override getState(): import("./world.js").SphericalJointState {
    const state = super.getState();
    if (!("distance" in state))
      throw new Error("Backend returned invalid spherical joint state");
    return state;
  }
}
export interface DistanceJointOptions extends JointOptions {
  readonly limits?: readonly [number, number];
}
export class DistanceJoint extends Joint<
  DistanceJointOptions & { readonly limits: readonly [number, number] }
> {
  constructor(options: DistanceJointOptions) {
    const limits = options.limits ?? [0, 0];
    validateLimits(limits);
    if (limits[0] < 0) throw new Error("Distance limits must be nonnegative");
    super({
      ...options,
      limits: Object.freeze<readonly [number, number]>([limits[0], limits[1]]),
    });
  }
  override getState(): import("./world.js").DistanceJointState {
    const state = super.getState();
    if (!("distance" in state))
      throw new Error("Backend returned invalid distance joint state");
    return state;
  }
  get limits(): readonly [number, number] {
    return this.options.limits;
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
