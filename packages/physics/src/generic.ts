import { readJoint } from "./reading.js";
import { Euler, Vector3 } from "three";
import { Joint, attach, sameLimits, validateLimits } from "./joint.js";
import type { JointOptions } from "./joint.js";
import type { JointDrive } from "./drive.js";
import type { AxisJointState } from "./world.js";

/** USD Physics degree-of-freedom tokens, in frame 0 coordinates. */
export type JointDof =
  "transX" | "transY" | "transZ" | "rotX" | "rotY" | "rotZ";
export const jointDofs: readonly JointDof[] = [
  "transX",
  "transY",
  "transZ",
  "rotX",
  "rotY",
  "rotZ",
];
export type DofMotion = "locked" | "free" | readonly [number, number];
export type GenericJointOptions = JointOptions & {
  /** Omitted dofs are locked. */
  readonly dofs?: Partial<Readonly<Record<JointDof, DofMotion>>>;
};
/** Six degrees of freedom, each locked, free, or limited, with a drive slot per axis. */
export class GenericJoint extends Joint<
  GenericJointOptions & {
    readonly dofs: Readonly<Record<JointDof, DofMotion>>;
  }
> {
  private readonly currentDrives = new Map<JointDof, JointDrive>();
  constructor(options: GenericJointOptions) {
    const motion = (axis: JointDof): DofMotion => {
      const value = options.dofs?.[axis] ?? "locked";
      if (typeof value === "string") return value;
      validateLimits(value);
      return [value[0], value[1]];
    };
    super({
      ...options,
      dofs: {
        transX: motion("transX"),
        transY: motion("transY"),
        transZ: motion("transZ"),
        rotX: motion("rotX"),
        rotY: motion("rotY"),
        rotZ: motion("rotZ"),
      },
    });
  }
  get dofs(): Readonly<Record<JointDof, DofMotion>> {
    return this.config.dofs;
  }
  getDrive(axis: JointDof): JointDrive | undefined {
    return this.currentDrives.get(axis);
  }
  setDrive(axis: JointDof, drive: JointDrive | undefined): this {
    this.assertLive();
    attach(this, this.currentDrives.get(axis), drive);
    if (drive) this.currentDrives.set(axis, drive);
    else this.currentDrives.delete(axis);
    this.touch();
    return this;
  }
  get drives(): ReadonlyMap<JointDof, JointDrive> {
    return this.currentDrives;
  }
  protected override releaseDrives(): void {
    for (const drive of this.currentDrives.values())
      attach(this, drive, undefined);
    this.currentDrives.clear();
  }
  protected override copyDrives(source: this): void {
    for (const axis of jointDofs)
      this.setDrive(axis, source.getDrive(axis)?.clone());
  }
  protected override sameConfiguration(source: this): boolean {
    return jointDofs.every((axis) => {
      const a = this.dofs[axis],
        b = source.dofs[axis];
      return typeof a === "string" || typeof b === "string"
        ? a === b
        : sameLimits(a, b);
    });
  }
  /** Rotations read as XYZ Euler angles of the relative rotation, wrapped. */
  getState(axis: JointDof): AxisJointState {
    const reading = readJoint(this);
    const index = jointDofs.indexOf(axis) % 3;
    if (axis.startsWith("trans"))
      return {
        position: reading.translation.getComponent(index),
        velocity: reading.linearVelocity.getComponent(index),
      };
    const euler = new Euler().setFromQuaternion(reading.rotation, "XYZ");
    return {
      position: new Vector3(euler.x, euler.y, euler.z).getComponent(index),
      velocity: reading.angularVelocity.getComponent(index),
    };
  }
}
