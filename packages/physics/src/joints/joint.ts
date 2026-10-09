import { Object3D, Matrix4, Vector3 } from "three";
import type { JointDrive } from "./drive.js";
import { RigidBody } from "../body.js";
import { constructLike } from "../construct.js";
import { assertRigidTransform, splitTransform } from "../transforms.js";
import { joinedWorld } from "../membership.js";
import type { PhysicsWorld } from "../world.js";

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
  readonly isPhysicsObject = true;
  /** Counts setting, drive and drive target changes, so backends reconcile only what changed. */
  version = 0;
  #options: Options;
  private currentEnabled = true;
  private currentCollideConnected = false;

  constructor(options: Options) {
    super();
    if (options.frame0) {
      assertRigidTransform(options.frame0);
      assertRigidTransform(options.frame1);
    }
    if (options.body0 === options.body1)
      throw new Error("Joint must connect distinct bodies");
    this.#options = {
      ...options,
      frame0: options.frame0?.clone(),
      frame1: options.frame1?.clone(),
    };
  }
  /** The world simulating this joint; set as the joint joins and leaves a world. */
  get world(): PhysicsWorld | undefined {
    return joinedWorld(this);
  }
  /** Fixed at construction and shared with clones. */
  get options(): Options {
    return this.#options;
  }
  get enabled(): boolean {
    return this.currentEnabled;
  }
  get collideConnected(): boolean {
    return this.currentCollideConnected;
  }
  setEnabled(value: boolean): this {
    this.currentEnabled = value;
    this.version++;
    return this;
  }
  setCollideConnected(value: boolean): this {
    this.currentCollideConnected = value;
    this.version++;
    return this;
  }

  /** Replaces the drive in a slot; a drive belongs to one joint at a time. */
  protected replaceDrive(
    previous: JointDrive | undefined,
    next: JointDrive | undefined,
  ): void {
    if (next && next !== previous && next.joint)
      throw new Error("Drive is already attached to a joint");
    if (previous) previous.joint = undefined;
    if (next) next.joint = this;
    this.version++;
  }
  /** Clones the source's drives into this joint's slots; joints with drive slots override. */
  protected copyDrives(_source: this): void {}

  /** A copy would still connect the original bodies; `clone(root)` reconnects it to cloned ones. */
  override clone(): this {
    throw new Error(
      "Joints clone with their bodies: use clone(root) from @drawcall/physics on an object holding both",
    );
  }

  /** Clones the joint onto the copies `objects` maps its bodies to. */
  cloneWithBodies(
    objects: ReadonlyMap<Object3D, Object3D>,
    recursive = true,
  ): this {
    const { body0, body1 } = this.options;
    return constructLike(this, [
      {
        ...this.options,
        body0: body0 && mappedBody(body0, objects),
        body1: mappedBody(body1, objects),
      },
    ]).copyState(this, recursive);
  }

  /** Copies the settings of a joint that shares these options, such as a clone. */
  override copy(source: this, recursive = true): this {
    if (source === this) return this;
    if (source.options !== this.options)
      throw new Error("Joint copy requires the same immutable options");
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
    const { body0, body1 } = this.options;
    body0?.updateWorldMatrix(true, false);
    body1.updateWorldMatrix(true, false);
    this.updateWorldMatrix(true, false);
    splitTransform(this.matrixWorld);
    if (body0) splitTransform(body0.matrixWorld);
    splitTransform(body1.matrixWorld);
  }

  getFrame(index: 0 | 1, target: Matrix4): Matrix4 {
    this.validate();
    const frame = index === 0 ? this.options.frame0 : this.options.frame1;
    const body = index === 0 ? this.options.body0 : this.options.body1;
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

export function validateLimits(value: readonly [number, number]): void {
  if (![value[0], value[1]].every(Number.isFinite) || value[0] > value[1])
    throw new Error("Invalid joint limits");
}

function mappedBody(
  body: RigidBody,
  objects: ReadonlyMap<Object3D, Object3D>,
): RigidBody {
  const target = objects.get(body) ?? body;
  if (!(target instanceof RigidBody))
    throw new Error("Joint copy requires bodies");
  return target;
}
