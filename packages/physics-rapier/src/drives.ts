import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  AxisJoint,
  DistanceJoint,
  GenericJoint,
  ScalarJoint,
  jointDofs,
  type Joint,
  type JointDof,
  type JointDrive,
  wrapAngle,
} from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import { bodyPose } from "./body.js";

type Axes = Record<"LinX" | "LinY" | "LinZ" | "AngX" | "AngY" | "AngZ", number>;

/** Rapier's `JointAxis` indices and `JointAxesMask` bits share member names, keyed here by `JointDof`. */
export function rapierDof(axes: Axes, dof: JointDof): number {
  const { LinX, LinY, LinZ, AngX, AngY, AngZ } = axes;
  return {
    transX: LinX,
    transY: LinY,
    transZ: LinZ,
    rotX: AngX,
    rotY: AngY,
    rotZ: AngZ,
  }[dof];
}

/** Every drive slot of a joint with its current drive: one for scalar joints, six for generic joints. */
function slots(object: Joint): [JointDof, JointDrive | undefined][] {
  if (object instanceof ScalarJoint)
    return [
      [object instanceof AxisJoint ? object.dof : "transX", object.drive],
    ];
  if (object instanceof GenericJoint)
    return jointDofs.map((axis) => [axis, object.getDrive(axis)]);
  return [];
}
export type DriveState = [JointDrive | undefined, number];
/** Which drive fills each slot and how it was last configured, for change detection. */
export function driveStates(object: Joint): DriveState[] {
  return slots(object).map(([, drive]) => [
    drive,
    drive?.settingsVersion ?? -1,
  ]);
}
export function sameDrives(a: DriveState[], b: DriveState[]): boolean {
  return (
    a.length === b.length &&
    a.every(
      ([drive, version], i) => b[i]?.[0] === drive && b[i]?.[1] === version,
    )
  );
}

/** Maps each slot's stiffness and damping onto Rapier's native motor; effort acts through `applyEfforts`. */
export function configureDrives(
  api: typeof Rapier,
  backend: Rapier.World,
  object: Joint,
  joint: Rapier.ImpulseJoint,
): void {
  for (const [axis, drive] of slots(object)) {
    if (drive?.options.maxVelocity !== undefined)
      throw new Error(
        "Rapier motors take a constant force limit, so they cannot model maxVelocity",
      );
    const goal = drive?.target;
    const gains =
      (drive?.options.stiffness ?? 0) > 0 || (drive?.options.damping ?? 0) > 0;
    // A Rapier motor without gains freezes the body; effort-only drives act through per-step forces instead.
    const native =
      drive && goal && gains ? { options: drive.options, goal } : undefined;
    const model: number =
      native?.options.model === "acceleration"
        ? api.MotorModel.AccelerationBased
        : api.MotorModel.ForceBased;
    const maxForce = native ? (native.options.maxForce ?? Number.MAX_VALUE) : 0;
    const position = native?.goal.position ?? 0;
    const settings = [
      // Rapier chooses the shortest arc and only wraps its error once.
      axis.startsWith("rot") ? wrapAngle(position) : position,
      native?.goal.velocity ?? 0,
      native?.options.stiffness ?? 0,
      native?.options.damping ?? 0,
    ] as const;
    if (joint instanceof api.UnitImpulseJoint) {
      joint.configureMotorModel(model);
      joint.setMotorMaxForce(maxForce);
      joint.configureMotor(...settings);
      continue;
    }
    const raw = backend.impulseJoints.raw;
    const index = rapierDof(api.JointAxis, axis);
    raw.jointConfigureMotorModel(joint.handle, index, model);
    raw.jointSetMotorMaxForce(joint.handle, index, maxForce);
    raw.jointConfigureMotor(joint.handle, index, ...settings);
  }
}

/** Applies each drive's effort term as a force pair for one step, capped by its `maxForce`. */
export function applyEfforts(object: Joint, joint: Rapier.ImpulseJoint): void {
  const efforts = slots(object).flatMap(([axis, drive]) =>
    drive?.target?.effort ? [{ axis, drive, effort: drive.target.effort }] : [],
  );
  if (!efforts.length) return;
  // Rapier numbers the two bodies from one.
  const body0 = joint.body1();
  const body1 = joint.body2();
  const anchor0 = new Vector3()
    .copy(joint.anchor1())
    .applyMatrix4(bodyPose(body0));
  const anchor1 = new Vector3()
    .copy(joint.anchor2())
    .applyMatrix4(bodyPose(body1));
  for (const { axis, drive, effort } of efforts) {
    const cap = drive.options.maxForce ?? Infinity;
    const value = Math.max(-cap, Math.min(cap, effort));
    let direction: Vector3;
    if (object instanceof DistanceJoint) {
      direction = anchor1.clone().sub(anchor0);
      if (direction.lengthSq() < 1e-24) continue;
      direction.normalize();
    } else {
      const index = jointDofs.indexOf(axis) % 3;
      direction = new Vector3()
        .setComponent(index, 1)
        .applyQuaternion(new Quaternion().copy(joint.frameX1()))
        .applyQuaternion(new Quaternion().copy(body0.rotation()));
    }
    const load = direction.multiplyScalar(value);
    if (axis.startsWith("rot")) {
      body1.addTorque(load, true);
      body0.addTorque(load.clone().negate(), true);
    } else {
      body1.addForceAtPoint(load, anchor1, true);
      body0.addForceAtPoint(load.clone().negate(), anchor0, true);
    }
  }
}
