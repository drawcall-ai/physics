import type { MainModule } from "@mujoco/mujoco";
import {
  splitTransform,
  type PhysicsVelocity,
  type RigidBody,
} from "@drawcall/physics";
import { setWorldPose, type Motion } from "@drawcall/physics/backend";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { Simulation } from "./model/compile.js";
import {
  array,
  at,
  pose,
  quaternion,
  rotation,
  vector,
  type HeapView,
} from "./heap.js";

export function bodyId(sim: Simulation, body: RigidBody): number {
  const id = sim.bodies.get(body);
  if (id === undefined) throw new Error("Missing MuJoCo rigid body");
  return id;
}
export function velocity(
  api: MainModule,
  sim: Simulation,
  id: number,
): PhysicsVelocity {
  const buffer = new api.DoubleBuffer(6);
  try {
    api.mj_objectVelocity(
      sim.model,
      sim.data,
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
/** The body's velocity at a world point, about its compiled center of mass. */
export function pointVelocity(
  api: MainModule,
  sim: Simulation,
  id: number,
  point: Vector3,
): Vector3 {
  const { linear, angular } = velocity(api, sim, id);
  const center = vector(sim.data.xipos, id * 3);
  return linear.add(angular.cross(point.clone().sub(center)));
}
/** Motion as the simulation measures it. */
export function motionOf(api: MainModule, sim: Simulation): Motion {
  return {
    angular: (body) => velocity(api, sim, bodyId(sim, body)).angular,
    velocityAt: (body, point) =>
      pointVelocity(api, sim, bodyId(sim, body), point),
  };
}
function freeJoint(api: MainModule, sim: Simulation, id: number): number {
  const joint = at(sim.model.body_jntadr, id);
  if (
    joint < 0 ||
    at(sim.model.jnt_type, joint) !== api.mjtJoint.mjJNT_FREE.value
  )
    throw new Error(
      "MuJoCo pose and velocity commands require a free root body; use joint drives to move articulated bodies",
    );
  return joint;
}
export function writeVelocity(
  api: MainModule,
  sim: Simulation,
  id: number,
  value: Partial<PhysicsVelocity>,
): void {
  const { model, data } = sim;
  const joint = freeJoint(api, sim, id);
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
  sim: Simulation,
  id: number,
  matrix: Matrix4,
): boolean {
  const { model, data } = sim;
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
  const address = at(model.jnt_qposadr, freeJoint(api, sim, id));
  return overwrite(array(data.qpos), [...position, ...q], address);
}
/**
 * Moves the bodies to the poses the scene holds for them. They move rigidly together, so each
 * articulation's change of pose goes onto its root; kinematic targets jump along.
 */
export function teleport(
  api: MainModule,
  sim: Simulation,
  bodies: ReadonlySet<RigidBody>,
): void {
  const { model, data } = sim;
  const roots = new Map<number, Matrix4>();
  for (const body of bodies) {
    const id = bodyId(sim, body);
    const matrix = splitTransform(body.matrixWorld).pose;
    const target = sim.targets.get(body);
    if (target !== undefined) writePose(api, sim, target, matrix);
    roots.set(
      at(model.body_rootid, id),
      matrix.clone().multiply(pose(data.xpos, data.xquat, id).invert()),
    );
  }
  for (const [root, delta] of roots)
    writePose(
      api,
      sim,
      root,
      delta.multiply(pose(data.xpos, data.xquat, root)),
    );
  api.mj_forward(model, data);
  writeBack(sim);
}
/** Writes the simulated poses of moving bodies back to the scene. */
export function writeBack(sim: Simulation): void {
  for (const [body, id] of sim.bodies)
    if (body.bodyType !== "static")
      setWorldPose(body, pose(sim.data.xpos, sim.data.xquat, id));
}

/**
 * Copies authored static and trigger poses into the model and recomputes derived state when
 * any of them, or a pose the caller already wrote (`moved`), changed.
 */
export function refreshPoses(
  api: MainModule,
  sim: Simulation,
  moved = false,
): void {
  for (const [body, id] of sim.bodies)
    if (body.bodyType === "static") {
      body.updateWorldMatrix(true, false);
      if (writePose(api, sim, id, splitTransform(body.matrixWorld).pose))
        moved = true;
    }
  for (const [trigger, id] of sim.triggers) {
    trigger.updateWorldMatrix(true, false);
    if (writePose(api, sim, id, splitTransform(trigger.matrixWorld).pose))
      moved = true;
  }
  if (moved) api.mj_forward(sim.model, sim.data);
}
