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
} from "@drawcall/physics";
import { wrapAngle } from "@drawcall/physics/backend";
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

/** Rejects drives Rapier's motors cannot model, before any native change. */
export function validateDrives(object: Joint): void {
  for (const [, drive] of slots(object)) {
    if (drive?.options.maxVelocity !== undefined)
      throw new Error(
        "Rapier motors take a constant force limit, so they cannot model maxVelocity",
      );
    if (
      drive?.options.maxForce !== undefined &&
      gains(drive) &&
      drive.target?.effort
    )
      throw new Error(
        "Rapier caps its motor and a drive effort separately, so a capped drive cannot combine gains with effort",
      );
  }
}

function gains(drive: JointDrive): boolean {
  return (drive.options.stiffness ?? 0) > 0 || (drive.options.damping ?? 0) > 0;
}

/** Maps each slot's stiffness and damping onto Rapier's native motor; effort acts through `applyDrives`. */
export function configureDrives(
  api: typeof Rapier,
  simulation: Rapier.World,
  object: Joint,
  joint: Rapier.ImpulseJoint,
): void {
  for (const [axis, drive] of slots(object)) {
    const goal = drive?.target;
    // A Rapier motor without gains freezes the body; effort-only drives act through per-step forces instead.
    const motor =
      drive && goal && gains(drive)
        ? { options: drive.options, goal }
        : undefined;
    const model: number =
      motor?.options.model === "acceleration"
        ? api.MotorModel.AccelerationBased
        : api.MotorModel.ForceBased;
    const maxForce = motor ? (motor.options.maxForce ?? Number.MAX_VALUE) : 0;
    const position = motor?.goal.position ?? 0;
    const settings = [
      // Rapier chooses the shortest arc and only wraps its error once.
      axis.startsWith("rot") ? wrapAngle(position) : position,
      motor?.goal.velocity ?? 0,
      motor?.options.stiffness ?? 0,
      motor?.options.damping ?? 0,
    ] as const;
    if (joint instanceof api.UnitImpulseJoint) {
      joint.configureMotorModel(model);
      joint.setMotorMaxForce(maxForce);
      joint.configureMotor(...settings);
      continue;
    }
    const raw = simulation.impulseJoints.raw;
    const index = rapierDof(api.JointAxis, axis);
    raw.jointConfigureMotorModel(joint.handle, index, model);
    raw.jointSetMotorMaxForce(joint.handle, index, maxForce);
    raw.jointConfigureMotor(joint.handle, index, ...settings);
  }
}

/** Applies each drive's effort term as a force pair for one step, capped by its `maxForce`. */
export function applyDrives(object: Joint, joint: Rapier.ImpulseJoint): void {
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
