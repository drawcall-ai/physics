import type { MainModule } from "@mujoco/mujoco";
import {
  splitTransform,
  type PhysicsVelocity,
  type RigidBody,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { Compiled } from "./model.js";
import { array, at, pose, quaternion, rotation, vector } from "./values.js";

export function bodyId(compiled: Compiled, body: RigidBody): number {
  const id = compiled.bodies.get(body);
  if (id === undefined) throw new Error("Missing compiled rigid body");
  return id;
}
export function velocity(
  api: MainModule,
  compiled: Compiled,
  id: number,
): PhysicsVelocity {
  const buffer = new api.DoubleBuffer(6);
  try {
    api.mj_objectVelocity(
      compiled.model,
      compiled.data,
      api.mjtObj.mjOBJ_BODY.value,
      id,
      buffer,
      0,
    );
    return {
      angular: vector(buffer.GetView(), 0),
      linear: vector(buffer.GetView(), 3),
    };
  } finally {
    buffer.delete();
  }
}
export function freeJoint(compiled: Compiled, id: number): number {
  const joint = at(compiled.model.body_jntadr, id);
  if (joint < 0 || at(compiled.model.jnt_type, joint) !== 0)
    throw new Error(
      "MuJoCo pose and velocity commands require a free root body; use joint drives to move articulated bodies",
    );
  return joint;
}
export function writeVelocity(
  compiled: Compiled,
  id: number,
  value: Partial<PhysicsVelocity>,
): void {
  const { model, data } = compiled;
  const joint = freeJoint(compiled, id);
  const address = at(model.jnt_dofadr, joint);
  const angular = value.angular
    ?.clone()
    .applyQuaternion(quaternion(data.xquat, id * 4).invert());
  if (angular) array(data.qvel).set(angular.toArray(), address + 3);
  if (value.linear) {
    const omega =
      value.angular ??
      vector(data.qvel, address + 3).applyQuaternion(
        quaternion(data.xquat, id * 4),
      );
    const offset = vector(data.xipos, id * 3).sub(vector(data.xpos, id * 3));
    array(data.qvel).set(
      value.linear.clone().sub(omega.cross(offset)).toArray(),
      address,
    );
  }
}
export function writePose(
  compiled: Compiled,
  id: number,
  matrix: Matrix4,
): void {
  const { model, data } = compiled;
  const position = new Vector3().setFromMatrixPosition(matrix).toArray();
  const q = rotation(new Quaternion().setFromRotationMatrix(matrix));
  const mocap = at(model.body_mocapid, id);
  if (mocap >= 0) {
    array(data.mocap_pos).set(position, mocap * 3);
    array(data.mocap_quat).set(q, mocap * 4);
  } else if (
    at(model.body_dofnum, id) === 0 &&
    at(model.body_parentid, id) === 0
  ) {
    array(model.body_pos).set(position, id * 3);
    array(model.body_quat).set(q, id * 4);
  } else {
    const address = at(model.jnt_qposadr, freeJoint(compiled, id));
    array(data.qpos).set([...position, ...q], address);
  }
}
export function wrench(
  api: MainModule,
  compiled: Compiled,
  id: number,
  force: Vector3,
  torque: Vector3,
  point: Vector3,
  impulse = false,
): void {
  const { model, data } = compiled;
  const buffer = new api.DoubleBuffer(model.nv);
  try {
    api.mj_applyFT(
      model,
      data,
      force.toArray(),
      torque.toArray(),
      point.toArray(),
      id,
      buffer,
    );
    const values = Array.from(array(buffer.GetView()));
    if (impulse) {
      api.mj_solveM(model, data, buffer, values);
      const delta = array(buffer.GetView());
      for (let i = 0; i < model.nv; i++)
        array(data.qvel)[i] = at(data.qvel, i) + at(delta, i);
      api.mj_forward(model, data);
    } else {
      for (let i = 0; i < model.nv; i++)
        array(data.qfrc_applied)[i] =
          at(data.qfrc_applied, i) + (values[i] ?? 0);
    }
  } finally {
    buffer.delete();
  }
}
export function synchronize(
  compiled: Compiled,
  set: (body: RigidBody, matrix: Matrix4) => void,
): void {
  for (const [body, id] of compiled.bodies)
    if (!body.disposed && body.bodyType !== "static")
      set(body, pose(compiled.data.xpos, compiled.data.xquat, id));
}

export function refreshPoses(api: MainModule, compiled: Compiled): void {
  for (const [body, id] of compiled.bodies)
    if (!body.disposed && body.bodyType === "static") {
      body.updateWorldMatrix(true, false);
      writePose(compiled, id, splitTransform(body.matrixWorld).pose);
    }
  for (const [trigger, id] of compiled.triggers)
    if (!trigger.disposed) {
      trigger.updateWorldMatrix(true, false);
      writePose(compiled, id, splitTransform(trigger.matrixWorld).pose);
    }
  api.mj_forward(compiled.model, compiled.data);
}
export function validateState(api: MainModule, compiled: Compiled): void {
  if (
    !array(compiled.data.qpos).every(Number.isFinite) ||
    !array(compiled.data.qvel).every(Number.isFinite)
  )
    throw new Error("MuJoCo produced non-finite simulation state");
  // Unlike contact, warning is a borrowed vector owned by MjData; deleting it corrupts the WASM heap.
  const warnings = compiled.data.warning;
  for (let i = 0; i < warnings.size(); i++) {
    const warning = warnings.get(i);
    if (!warning) throw new Error("Missing MuJoCo warning state");
    try {
      if (warning.number > 0)
        throw new Error(api.mju_warningText(i, warning.lastinfo));
    } finally {
      warning.delete();
    }
  }
}
