import { readJoint } from "./reading.js";
import { Joint, attach, sameLimits, validateLimits } from "./joint.js";
import type { JointOptions } from "./joint.js";
import type { JointDrive } from "./drive.js";
import type {
  AxisJointState,
  SphericalJointState,
  DistanceJointState,
} from "./world.js";

export class FixedJoint extends Joint {}

/** A joint with one scalar coordinate and one drive slot. */
export abstract class ScalarJoint<
  Options extends JointOptions = JointOptions,
> extends Joint<Options> {
  private currentDrive?: JointDrive;
  get drive(): JointDrive | undefined {
    return this.currentDrive;
  }
  setDrive(drive: JointDrive | undefined): this {
    this.assertLive();
    attach(this, this.currentDrive, drive);
    this.currentDrive = drive;
    this.touch();
    return this;
  }
  protected override releaseDrives(): void {
    attach(this, this.currentDrive, undefined);
    this.currentDrive = undefined;
  }
  protected override copyDrives(source: this): void {
    this.setDrive(source.drive?.clone());
  }
  abstract getState(): AxisJointState | DistanceJointState;
}

export type AxisJointOptions = JointOptions & {
  readonly axis?: "X" | "Y" | "Z";
  readonly limits?: readonly [number, number];
};
export abstract class AxisJoint extends ScalarJoint<
  AxisJointOptions & { readonly axis: "X" | "Y" | "Z" }
> {
  /** The joint coordinate its drive acts on. */
  abstract readonly dof: "rotX" | "transX";
  constructor(options: AxisJointOptions) {
    if (options.limits) validateLimits(options.limits);
    super({
      ...options,
      axis: options.axis ?? "Y",
      limits: options.limits && [...options.limits],
    });
  }
  get limits(): readonly [number, number] | undefined {
    return this.config.limits;
  }
  protected override sameConfiguration(source: this): boolean {
    return (
      this.config.axis === source.config.axis &&
      sameLimits(this.limits, source.limits)
    );
  }
  getState(): AxisJointState {
    const reading = readJoint(this);
    return this.dof === "rotX"
      ? { position: reading.angle, velocity: reading.angularVelocity.x }
      : {
          position: reading.translation.x,
          velocity: reading.linearVelocity.x,
        };
  }
}
export class RevoluteJoint extends AxisJoint {
  readonly dof = "rotX";
}
export class PrismaticJoint extends AxisJoint {
  readonly dof = "transX";
}
export class SphericalJoint extends Joint {
  getState(): SphericalJointState {
    const { rotation, angularVelocity } = readJoint(this);
    return { rotation, angularVelocity };
  }
}
export type DistanceJointOptions = JointOptions & {
  readonly limits?: readonly [number, number];
};
export class DistanceJoint extends ScalarJoint<
  DistanceJointOptions & { readonly limits: readonly [number, number] }
> {
  constructor(options: DistanceJointOptions) {
    const limits = options.limits ?? [0, 0];
    if (!(
      Number.isFinite(limits[0]) &&
      limits[0] >= 0 &&
      limits[1] >= limits[0]
    ))
      throw new Error(
        "Distance limits need a finite nonnegative minimum and a maximum of at least the minimum",
      );
    super({
      ...options,
      limits: [limits[0], limits[1]],
    });
  }
  /** `Infinity` leaves the distance free, so a drive alone acts as a spring. */
  get limits(): readonly [number, number] {
    return this.config.limits;
  }
  protected override sameConfiguration(source: this): boolean {
    return sameLimits(this.limits, source.limits);
  }
  getState(): DistanceJointState {
    const { translation, linearVelocity } = readJoint(this);
    const distance = translation.length();
    return {
      distance,
      velocity:
        distance > 1e-12 ? linearVelocity.dot(translation) / distance : 0,
    };
  }
}
