import type { Joint } from "./joint.js";
import { constructLike } from "../construct.js";

export interface JointDriveOptions {
  /** N/m for translations, N·m/rad for rotations. */
  readonly stiffness?: number;
  readonly damping?: number;
  /** Cap on the drive force; unbounded when omitted. */
  readonly maxForce?: number;
  /**
   * The motor's no-load speed, where its `maxForce` is spent on its own back-EMF. It damps the
   * coordinate by `maxForce / maxVelocity`, so a saturated drive settles at this speed instead of
   * accelerating without limit. Needs `maxForce`; unlimited when omitted.
   */
  readonly maxVelocity?: number;
  /** `acceleration` scales gains by the driven mass; backends without it reject the drive. */
  readonly model?: "force" | "acceleration";
}
export interface JointDriveTarget {
  readonly position: number;
  readonly velocity: number;
  readonly effort: number;
}
/**
 * The force law on one joint coordinate:
 * `stiffness · (position − q) + damping · (velocity − q̇) + effort`, capped by `maxForce`.
 * Options are fixed at construction; the target changes at runtime. Without a target the
 * drive exerts no force. A drive with a constant target is a spring.
 */
export class JointDrive<Options extends JointDriveOptions = JointDriveOptions> {
  readonly options: Options;
  /** The joint this drive is attached to; set by the joint's `setDrive`. */
  joint: Joint | undefined = undefined;
  private currentTarget?: JointDriveTarget;
  constructor(options: Options) {
    for (const value of [
      options.stiffness,
      options.damping,
      options.maxForce,
    ]) {
      if (value !== undefined && (!Number.isFinite(value) || value < 0))
        throw new Error(
          "Drive gains and maximum force must be finite and nonnegative",
        );
    }
    const { maxVelocity, maxForce } = options;
    if (maxVelocity !== undefined) {
      if (!Number.isFinite(maxVelocity) || maxVelocity <= 0)
        throw new Error("Drive maximum velocity must be finite and positive");
      if (maxForce === undefined || !Number.isFinite(maxForce))
        throw new Error("A drive velocity limit needs a finite maximum force");
    }
    this.options = options;
  }

  /** The damping `maxForce / maxVelocity` the motor exerts on its own coordinate; undefined when unlimited. */
  get backEmf(): number | undefined {
    const { maxForce, maxVelocity } = this.options;
    if (maxVelocity === undefined || maxForce === undefined) return undefined;
    return maxForce / maxVelocity;
  }
  get target(): JointDriveTarget | undefined {
    return this.currentTarget;
  }
  /**
   * Replaces the whole target; omitted terms are zero. `undefined` makes the drive passive. A
   * change counts as a change of the attached joint.
   */
  setTarget(target: Partial<JointDriveTarget> | undefined): this {
    this.currentTarget = target && this.complete(target);
    if (this.joint) this.joint.version++;
    return this;
  }
  private complete(target: Partial<JointDriveTarget>): JointDriveTarget {
    const next = {
      position: target.position ?? 0,
      velocity: target.velocity ?? 0,
      effort: target.effort ?? 0,
    };
    if (!Object.values(next).every(Number.isFinite))
      throw new Error("Drive targets must be finite");
    if (next.position !== 0 && !this.options.stiffness)
      throw new Error("A position target needs stiffness");
    if (next.velocity !== 0 && !this.options.damping)
      throw new Error("A velocity target needs damping");
    return next;
  }
  clone(): this {
    return constructLike(this, [this.options]).copy(this);
  }
  copy(source: this): this {
    return this.setTarget(source.target);
  }
}
