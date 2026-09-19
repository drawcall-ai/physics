import { disposeClonedPhysics } from "./clone.js";
import { Object3D, Matrix4, Vector3 } from "three";
import { JointDrive, bindDrive } from "./drive.js";
import { RigidBody } from "./body.js";
import { constructLike } from "./construct.js";
import { cleanup, rollback } from "./cleanup.js";
import { assertRigidTransform, splitTransform } from "./transforms.js";
import { registry } from "./registry.js";

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
  /** Marks a settings change for backends to reconcile before the next step. */
  protected touch(): void {
    this.version++;
  }
  setEnabled(value: boolean): this {
    this.assertLive();
    this.currentEnabled = value;
    this.touch();
    return this;
  }
  setCollideConnected(value: boolean): this {
    this.assertLive();
    this.currentCollideConnected = value;
    this.touch();
    return this;
  }

  constructor(options: Options) {
    super();
    this.config = {
      ...options,
      frame0: options.frame0?.clone(),
      frame1: options.frame1?.clone(),
    };
    if (options.frame0) {
      assertRigidTransform(options.frame0);
      assertRigidTransform(options.frame1);
    }
    if (options.body0 === options.body1)
      throw new Error("Joint must connect distinct bodies");
    if (options.body0?.disposed || options.body1.disposed)
      throw new Error("Joint cannot connect disposed bodies");
    registry.register(this);
  }

  get disposed(): boolean {
    return this.isDisposed;
  }
  /** Whether `body` is either side of this joint. */
  connects(body: RigidBody): boolean {
    return this.config.body0 === body || this.config.body1 === body;
  }

  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    cleanup(
      [
        () => this.releaseDrives(),
        () => registry.unregister(this),
        () => this.removeFromParent(),
      ],
      "Joint disposal failed",
    );
  }
  /** Detaches every drive; joints with drive slots override. */
  protected releaseDrives(): void {}
  /** Clones the source's drives into this joint's slots; joints with drive slots override. */
  protected copyDrives(_source: this): void {}
  /** Immutable options beyond bodies and frames that a copy must share. */
  protected sameConfiguration(_source: this): boolean {
    return true;
  }

  override clone(recursive = true): this {
    return this.cloneWithBodies(new Map(), recursive);
  }

  cloneWithBodies(
    objects: ReadonlyMap<Object3D, Object3D>,
    recursive = true,
  ): this {
    if (this.disposed) throw new Error("Cannot clone a disposed joint");
    const target = constructLike(this, [
      {
        ...this.config,
        body0: this.config.body0
          ? mappedBody(this.config.body0, objects)
          : null,
        body1: mappedBody(this.config.body1, objects),
      },
    ]);
    try {
      return objects.size
        ? target.copyState(this, recursive)
        : target.copy(this, recursive);
    } catch (error) {
      rollback(
        error,
        [() => disposeClonedPhysics(target)],
        "Joint clone failed",
      );
    }
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
      !this.sameConfiguration(source)
    )
      throw new Error("Joint copy requires matching immutable options");
    return this.copyState(source, recursive);
  }

  private copyState(source: this, recursive: boolean): this {
    super.copy(source, recursive);
    this.setEnabled(source.enabled).setCollideConnected(
      source.collideConnected,
    );
    this.copyDrives(source);
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

/** Moves a drive between slots; a drive belongs to one joint at a time. */
export function attach(
  joint: Joint,
  previous: JointDrive | undefined,
  next: JointDrive | undefined,
): void {
  if (next && next !== previous && next.joint)
    throw new Error("Drive is already attached to a joint");
  if (previous && previous !== next) bindDrive(previous, undefined);
  if (next) bindDrive(next, joint);
}

export function validateLimits(value: readonly [number, number]): void {
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
  if (!(target instanceof RigidBody) || target.disposed)
    throw new Error("Joint copy requires live bodies");
  return target;
}

export function sameLimits(
  a: readonly [number, number] | undefined,
  b: readonly [number, number] | undefined,
): boolean {
  return a === undefined
    ? b === undefined
    : b !== undefined && a[0] === b[0] && a[1] === b[1];
}
