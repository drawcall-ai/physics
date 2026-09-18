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
import type { Compiled } from "./model.js";
import { driveOf, unconstrained, type JointRecord } from "./joints.js";
import { at, array, vector, quaternion } from "./values.js";
import { bodyId, wrench, velocity } from "./motion.js";

export function applyDrives(
  api: MainModule,
  compiled: Compiled,
  joints: ReadonlyMap<Joint, JointRecord>,
  read: (joint: Joint) => JointReading,
  dt: number,
): void {
  const { model, data } = compiled;
  for (const [id, coordinate] of compiled.coordinates) {
    const actuator = api.mj_name2id(
      model,
      api.mjtObj.mjOBJ_ACTUATOR.value,
      `${coordinate.name}drive`,
    );
    const drive = driveOf(coordinate);
    const target = drive?.target;
    const scale =
      drive?.options.model === "acceleration"
        ? effectiveMass(api, compiled, at(model.jnt_dofadr, id))
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
    const force = new Vector3(),
      torque = new Vector3();
    if (joint instanceof GenericJoint)
      for (const [i, axis] of jointDofs.entries()) {
        const drive = joint.getDrive(axis);
        const state = joint.getState(axis);
        (i < 3 ? force : torque).setComponent(
          i % 3,
          effort(
            drive,
            state.position,
            state.velocity,
            at(model.body_mass, bodyId(compiled, body1)),
            dt,
          ),
        );
      }
    if (joint instanceof DistanceJoint) {
      const distance = reading.translation.length();
      const speed =
        distance > 1e-12
          ? reading.linearVelocity.dot(reading.translation) / distance
          : 0;
      force
        .copy(reading.translation)
        .normalize()
        .multiplyScalar(
          effort(
            joint.drive,
            distance,
            speed,
            at(model.body_mass, bodyId(compiled, body1)),
            dt,
          ),
        );
    }
    force.applyQuaternion(rotation);
    torque.applyQuaternion(rotation);
    wrench(
      api,
      compiled,
      bodyId(compiled, body1),
      force,
      torque,
      new Vector3().setFromMatrixPosition(frame1),
    );
    if (body0)
      wrench(
        api,
        compiled,
        bodyId(compiled, body0),
        force.negate(),
        torque.negate(),
        new Vector3().setFromMatrixPosition(frame0),
      );
  }
  for (const [body, id] of compiled.bodies) {
    if (body.bodyType !== "dynamic") continue;
    array(model.body_gravcomp)[id] = 1 - body.gravityScale;
    if (!body.linearDamping && !body.angularDamping) continue;
    const v = velocity(api, compiled, id);
    const force = v.linear.multiplyScalar(
      (-body.linearDamping * at(model.body_mass, id)) /
        (1 + body.linearDamping * dt),
    );
    const rotation = quaternion(data.xquat, id * 4).multiply(
      quaternion(model.body_iquat, id * 4),
    );
    const torque = v.angular
      .applyQuaternion(rotation.clone().invert())
      .multiply(vector(model.body_inertia, id * 3))
      .applyQuaternion(rotation)
      .multiplyScalar(-body.angularDamping / (1 + body.angularDamping * dt));
    wrench(api, compiled, id, force, torque, vector(data.xipos, id * 3));
  }
}
function effectiveMass(
  api: MainModule,
  compiled: Compiled,
  index: number,
): number {
  const values = new Array<number>(compiled.model.nv).fill(0);
  values[index] = 1;
  const buffer = new api.DoubleBuffer(values.length);
  try {
    api.mj_solveM(compiled.model, compiled.data, buffer, values);
    return 1 / at(buffer.GetView(), index);
  } finally {
    buffer.delete();
  }
}
function effort(
  drive: JointDrive | undefined,
  position: number,
  velocity: number,
  mass: number,
  dt: number,
): number {
  if (!drive?.target) return 0;
  const { target, options } = drive;
  const scale = options.model === "acceleration" ? mass : 1;
  const k = (options.stiffness ?? 0) * scale,
    d = (options.damping ?? 0) * scale;
  const force =
    (k * (target.position - position) + d * (target.velocity - velocity)) /
      (1 + (d * dt + k * dt * dt) / mass) +
    target.effort;
  const max = options.maxForce ?? Infinity;
  return Math.max(-max, Math.min(max, force));
}
