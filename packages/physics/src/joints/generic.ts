import { readJoint } from "../world.js";
import type { AxisJointState, JointPose, JointReading } from "./reading.js";
import { Euler, Vector3 } from "three";
import { Joint, validateLimits } from "./joint.js";
import type { JointOptions } from "./joint.js";
import type { JointDrive } from "./drive.js";

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
      return value;
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
    return this.options.dofs;
  }
  getDrive(axis: JointDof): JointDrive | undefined {
    return this.currentDrives.get(axis);
  }
  setDrive(axis: JointDof, drive: JointDrive | undefined): this {
    this.replaceDrive(this.currentDrives.get(axis), drive);
    if (drive) this.currentDrives.set(axis, drive);
    else this.currentDrives.delete(axis);
    return this;
  }
  get drives(): ReadonlyMap<JointDof, JointDrive> {
    return this.currentDrives;
  }
  protected override copyDrives(source: this): void {
    for (const axis of jointDofs)
      this.setDrive(axis, source.getDrive(axis)?.clone());
  }
  getState(axis: JointDof): AxisJointState {
    return dofState(readJoint(this), axis);
  }
}

/** One degree of freedom of a reading; rotations read as XYZ Euler angles of the relative rotation, wrapped. */
export function dofState(
  reading: JointReading,
  axis: JointDof,
): AxisJointState {
  const index = jointDofs.indexOf(axis) % 3;
  const velocity = axis.startsWith("trans")
    ? reading.linearVelocity
    : reading.angularVelocity;
  return {
    position: dofPosition(reading, axis),
    velocity: velocity.getComponent(index),
  };
}

/** The position of one degree of freedom, as `dofState` reads it. */
export function dofPosition(pose: JointPose, axis: JointDof): number {
  const index = jointDofs.indexOf(axis) % 3;
  if (axis.startsWith("trans")) return pose.translation.getComponent(index);
  const euler = new Euler().setFromQuaternion(pose.rotation, "XYZ");
  return new Vector3(euler.x, euler.y, euler.z).getComponent(index);
}
