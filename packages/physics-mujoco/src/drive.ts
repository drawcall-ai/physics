import type { MainModule } from "@mujoco/mujoco";
import {
  GenericJoint,
  DistanceJoint,
  jointDofs,
  splitTransform,
  type Joint,
  type JointDrive,
} from "@drawcall/physics";
import {
  dofState,
  unconstrained,
  type JointBinding,
} from "@drawcall/physics/backend";
import { Quaternion, Vector3 } from "three";
import type { Simulation } from "./model/compile.js";
import { driveOf } from "./model/joints.js";
import { array } from "./heap.js";
import { bodyId, motionOf } from "./body.js";
import { project, inverseInertia, apply, type Load } from "./force.js";

/**
 * Sets the actuators that drive tree-joint coordinates, and applies the drives of distance and
 * unconstrained joints, which have no coordinate, as generalized forces.
 */
export function applyDrives(
  api: MainModule,
  sim: Simulation,
  joints: ReadonlyMap<Joint, JointBinding>,
  dt: number,
): void {
  configureActuators(api, sim);
  const motion = motionOf(api, sim);
  for (const [joint, binding] of joints) {
    if (
      !joint.enabled ||
      (!(joint instanceof DistanceJoint) && !unconstrained(joint))
    )
      continue;
    const reading = binding.read(motion);
    const body0 = joint.options.body0,
      body1 = joint.options.body1;
    const frame0 = binding.frames[0].clone();
    if (body0) frame0.premultiply(splitTransform(body0.matrixWorld).pose);
    const frame1 = binding.frames[1]
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
      if (!drive || (!drive.target && drive.backEmf === undefined)) return;
      direction.applyQuaternion(rotation);
      const force = angular ? new Vector3() : direction;
      const torque = angular ? direction : new Vector3();
      const loads: Load[] = [
        { body: bodyId(sim, body1), force, torque, point },
      ];
      // Frame-0 translation rotates about anchor 0: its reaction also acts at anchor 1.
      if (body0)
        loads.push({
          body: bodyId(sim, body0),
          force: force.clone().negate(),
          torque: torque.clone().negate(),
          point,
        });
      const generalized = project(api, sim, loads);
      const inverse = inverseInertia(api, sim, generalized);
      if (inverse === 0) return;
      apply(sim, generalized, effort(drive, position, speed, inverse, dt));
    };
    if (joint instanceof GenericJoint)
      for (const [i, axis] of jointDofs.entries()) {
        const state = dofState(reading, axis);
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

function configureActuators(api: MainModule, sim: Simulation): void {
  const { model, data } = sim;
  for (const coordinate of sim.coordinates.values()) {
    const { actuator, dof } = coordinate;
    const drive = driveOf(coordinate);
    const target = drive?.target;
    const scale =
      drive?.options.model === "acceleration"
        ? coordinateInertia(api, sim, dof)
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
  sim: Simulation,
  index: number,
): number {
  const direction = new Float64Array(sim.model.nv);
  if (index < 0 || index >= direction.length)
    throw new Error("Missing actuator coordinate");
  direction[index] = 1;
  const inverse = inverseInertia(api, sim, direction);
  if (inverse === 0)
    throw new Error("Actuator coordinate has no inertial response");
  return 1 / inverse;
}
function effort(
  drive: JointDrive,
  position: number,
  velocity: number,
  inverse: number,
  dt: number,
): number {
  const { target, options } = drive;
  const scale = options.model === "acceleration" ? 1 / inverse : 1;
  const k = target ? (options.stiffness ?? 0) * scale : 0,
    d = target ? (options.damping ?? 0) * scale : 0;
  const force = target
    ? (k * (target.position - position - dt * velocity) +
        d * (target.velocity - velocity) +
        target.effort) /
      (1 + (d * dt + k * dt * dt) * inverse)
    : 0;
  const max = options.maxForce ?? Infinity;
  // Back-EMF brakes the coordinate whenever the motor is connected and sits outside the force cap,
  // as the joint damping does for tree joints.
  const emf = drive.backEmf ?? 0;
  return (
    Math.max(-max, Math.min(max, force)) -
    (emf * velocity) / (1 + emf * dt * inverse)
  );
}
