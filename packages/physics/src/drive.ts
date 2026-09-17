import type { Joint } from "./joint.js";
import { constructLike } from "./construct.js";

export interface JointDriveOptions {
  /** N/m for translations, N·m/rad for rotations. */
  readonly stiffness?: number;
  readonly damping?: number;
  /** Cap on the drive force; unbounded when omitted. */
  readonly maxForce?: number;
  /** `acceleration` scales gains by the driven mass; backends without it reject the drive. */
  readonly model?: "force" | "acceleration";
}
export interface JointDriveTarget {
  readonly position: number;
  readonly velocity: number;
  readonly effort: number;
}
const attachments = new WeakMap<JointDrive, Joint>();

/**
 * The force law on one joint coordinate:
 * `stiffness · (position − q) + damping · (velocity − q̇) + effort`, capped by `maxForce`.
 * Options are fixed at construction; the target changes at runtime. Without a target the
 * drive exerts no force. A drive with a constant target is a spring.
 */
export class JointDrive<Options extends JointDriveOptions = JointDriveOptions> {
  readonly options: Options;
  private currentTarget?: JointDriveTarget;
  private version = 0;
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
    this.options = { ...options };
  }
  /** The joint this drive is attached to through its `setDrive`. */
  get joint(): Joint | undefined {
    return attachments.get(this);
  }
  get target(): JointDriveTarget | undefined {
    return this.currentTarget;
  }
  get settingsVersion(): number {
    return this.version;
  }
  /** Replaces the whole target; omitted terms are zero. `undefined` makes the drive passive. */
  setTarget(target: Partial<JointDriveTarget> | undefined): this {
    if (target === undefined) {
      this.currentTarget = undefined;
      this.version++;
      return this;
    }
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
    this.currentTarget = next;
    this.version++;
    return this;
  }
  clone(): this {
    return constructLike(this, [this.options]).copy(this);
  }
  copy(source: this): this {
    return this.setTarget(source.target);
  }
}

/** Joint integration: attachment changes only through a joint's `setDrive`. */
export function bindDrive(drive: JointDrive, joint: Joint | undefined): void {
  if (joint) attachments.set(drive, joint);
  else attachments.delete(drive);
}
