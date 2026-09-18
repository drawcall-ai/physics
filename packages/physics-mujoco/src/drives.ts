import type { MainModule } from "@mujoco/mujoco";
import {
  GenericJoint,
  DistanceJoint,
  jointDofs,
  splitTransform,
  type Joint,
  type JointDrive,
  type JointReading,
} from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import type { Compiled } from "./model/compile.js";
import { driveOf, unconstrained, type JointRecord } from "./model/joints.js";
import { array } from "./values.js";
import { bodyId } from "./motion.js";
import { project, inverseInertia, apply, type Load } from "./forces.js";

export function applyDrives(
  api: MainModule,
  compiled: Compiled,
  joints: ReadonlyMap<Joint, JointRecord>,
  read: (joint: Joint) => JointReading,
  dt: number,
): void {
  configureActuators(api, compiled);
  for (const [joint, record] of joints) {
    if (
      !joint.enabled ||
      (!(joint instanceof DistanceJoint) && !unconstrained(joint))
    )
      continue;
    const reading = read(joint);
    const body0 = joint.options.body0,
      body1 = joint.options.body1;
    const frame0 = record.frames[0].clone();
    if (body0) frame0.premultiply(splitTransform(body0.matrixWorld).pose);
    const frame1 = record.frames[1]
      .clone()
      .premultiply(splitTransform(body1.matrixWorld).pose);
    const rotation = new Quaternion().setFromRotationMatrix(frame0);
    const point = new Vector3().setFromMatrixPosition(frame1);
    const driveAxis = (
      drive: JointDrive | undefined,
      position: number,
      speed: number,
      direction: Vector3,
      angular: boolean,
    ) => {
      if (!drive?.target) return;
      direction.applyQuaternion(rotation);
      const force = angular ? new Vector3() : direction;
      const torque = angular ? direction : new Vector3();
      const loads: Load[] = [
        { body: bodyId(compiled, body1), force, torque, point },
      ];
      // Frame-0 translation rotates about anchor 0: its reaction also acts at anchor 1.
      if (body0)
        loads.push({
          body: bodyId(compiled, body0),
          force: force.clone().negate(),
          torque: torque.clone().negate(),
          point,
        });
      const generalized = project(api, compiled, loads);
      const inverse = inverseInertia(api, compiled, generalized);
      if (inverse === 0) return;
      apply(compiled, generalized, effort(drive, position, speed, inverse, dt));
    };
    if (joint instanceof GenericJoint)
      for (const [i, axis] of jointDofs.entries()) {
        const state = joint.getState(axis);
        driveAxis(
          joint.getDrive(axis),
          state.position,
          state.velocity,
          new Vector3().setComponent(i % 3, 1),
          i >= 3,
        );
      }
    if (joint instanceof DistanceJoint) {
      const distance = reading.translation.length();
      if (distance === 0) continue;
      driveAxis(
        joint.drive,
        distance,
        reading.linearVelocity.dot(reading.translation) / distance,
        reading.translation.clone().divideScalar(distance),
        false,
      );
    }
  }
}

function configureActuators(api: MainModule, compiled: Compiled): void {
  const { model, data } = compiled;
  for (const coordinate of compiled.coordinates.values()) {
    const { actuator, dof } = coordinate;
    const drive = driveOf(coordinate);
    const target = drive?.target;
    const scale =
      drive?.options.model === "acceleration"
        ? coordinateInertia(api, compiled, dof)
        : 1;
    const stiffness = target ? (drive?.options.stiffness ?? 0) * scale : 0;
    const damping = target ? (drive?.options.damping ?? 0) * scale : 0;
    array(model.actuator_biasprm)[actuator * 10 + 1] = -stiffness;
    array(model.actuator_biasprm)[actuator * 10 + 2] = -damping;
    array(model.actuator_forcerange).set(
      [-(drive?.options.maxForce ?? 0), drive?.options.maxForce ?? 0],
      actuator * 2,
    );
    array(data.ctrl)[actuator] = target
      ? stiffness * target.position + damping * target.velocity + target.effort
      : 0;
  }
}
function coordinateInertia(
  api: MainModule,
  compiled: Compiled,
  index: number,
): number {
  const direction = new Float64Array(compiled.model.nv);
  if (index < 0 || index >= direction.length)
    throw new Error("Missing actuator coordinate");
  direction[index] = 1;
  const inverse = inverseInertia(api, compiled, direction);
  if (inverse === 0)
    throw new Error("Actuator coordinate has no inertial response");
  return 1 / inverse;
}
function effort(
  drive: JointDrive | undefined,
  position: number,
  velocity: number,
  inverse: number,
  dt: number,
): number {
  if (!drive?.target) return 0;
  const { target, options } = drive;
  const scale = options.model === "acceleration" ? 1 / inverse : 1;
  const k = (options.stiffness ?? 0) * scale,
    d = (options.damping ?? 0) * scale;
  const force =
    (k * (target.position - position - dt * velocity) +
      d * (target.velocity - velocity) +
      target.effort) /
    (1 + (d * dt + k * dt * dt) * inverse);
  const max = options.maxForce ?? Infinity;
  return Math.max(-max, Math.min(max, force));
}
