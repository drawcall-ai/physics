import type { MainModule } from "@mujoco/mujoco";
import {
  splitTransform,
  type PhysicsVelocity,
  type RigidBody,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { Compiled } from "./model/compile.js";
import {
  array,
  at,
  pose,
  quaternion,
  rotation,
  vector,
  type HeapView,
} from "./values.js";

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
export function freeJoint(
  api: MainModule,
  compiled: Compiled,
  id: number,
): number {
  const joint = at(compiled.model.body_jntadr, id);
  if (
    joint < 0 ||
    at(compiled.model.jnt_type, joint) !== api.mjtJoint.mjJNT_FREE.value
  )
    throw new Error(
      "MuJoCo pose and velocity commands require a free root body; use joint drives to move articulated bodies",
    );
  return joint;
}
export function writeVelocity(
  api: MainModule,
  compiled: Compiled,
  id: number,
  value: Partial<PhysicsVelocity>,
): void {
  const { model, data } = compiled;
  const joint = freeJoint(api, compiled, id);
  const address = at(model.jnt_dofadr, joint);
  const orientation = quaternion(data.xquat, id * 4);
  const offset = vector(data.xipos, id * 3).sub(vector(data.xpos, id * 3));
  const previousAngular = vector(data.qvel, address + 3).applyQuaternion(
    orientation,
  );
  const linear =
    value.linear?.clone() ??
    vector(data.qvel, address).add(previousAngular.clone().cross(offset));
  const angular = value.angular?.clone() ?? previousAngular;
  array(data.qvel).set(
    linear.sub(angular.clone().cross(offset)).toArray(),
    address,
  );
  array(data.qvel).set(
    angular.applyQuaternion(orientation.invert()).toArray(),
    address + 3,
  );
}
/** Overwrite a span and report whether it held different values. */
function overwrite(
  target: HeapView,
  values: readonly number[],
  offset: number,
): boolean {
  let moved = false;
  for (const [i, value] of values.entries()) {
    if (target[offset + i] === value) continue;
    target[offset + i] = value;
    moved = true;
  }
  return moved;
}
/** Returns whether the pose differed from the one already stored. */
export function writePose(
  api: MainModule,
  compiled: Compiled,
  id: number,
  matrix: Matrix4,
): boolean {
  const { model, data } = compiled;
  const position = new Vector3().setFromMatrixPosition(matrix).toArray();
  const q = rotation(new Quaternion().setFromRotationMatrix(matrix));
  const mocap = at(model.body_mocapid, id);
  if (mocap >= 0) {
    const moved = overwrite(array(data.mocap_pos), position, mocap * 3);
    return overwrite(array(data.mocap_quat), q, mocap * 4) || moved;
  }
  if (at(model.body_dofnum, id) === 0 && at(model.body_parentid, id) === 0) {
    const moved = overwrite(array(model.body_pos), position, id * 3);
    return overwrite(array(model.body_quat), q, id * 4) || moved;
  }
  const address = at(model.jnt_qposadr, freeJoint(api, compiled, id));
  return overwrite(array(data.qpos), [...position, ...q], address);
}
export function synchronize(
  compiled: Compiled,
  set: (body: RigidBody, matrix: Matrix4) => void,
): void {
  for (const [body, id] of compiled.bodies)
    if (!body.disposed && body.bodyType !== "static")
      set(body, pose(compiled.data.xpos, compiled.data.xquat, id));
}

/**
 * Copies authored static and trigger poses into the model and recomputes derived state when
 * any of them, or a pose the caller already wrote (`moved`), changed.
 */
export function refreshPoses(
  api: MainModule,
  compiled: Compiled,
  moved = false,
): void {
  for (const [body, id] of compiled.bodies)
    if (!body.disposed && body.bodyType === "static") {
      body.updateWorldMatrix(true, false);
      if (writePose(api, compiled, id, splitTransform(body.matrixWorld).pose))
        moved = true;
    }
  for (const [trigger, id] of compiled.triggers)
    if (!trigger.disposed) {
      trigger.updateWorldMatrix(true, false);
      if (
        writePose(api, compiled, id, splitTransform(trigger.matrixWorld).pose)
      )
        moved = true;
    }
  if (moved) api.mj_forward(compiled.model, compiled.data);
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
