import type { MainModule } from "@mujoco/mujoco";
import { Vector3 } from "three";
import type { Simulation } from "./model/compile.js";
import { array, at, vector, quaternion } from "./heap.js";
import { velocity } from "./body.js";

export interface Load {
  body: number;
  force: Vector3;
  torque: Vector3;
  point: Vector3;
}
/** Project world-space loads into generalized forces, including every body's lever arm. */
export function project(
  api: MainModule,
  sim: Simulation,
  loads: readonly Load[],
): Float64Array {
  const buffer = new api.DoubleBuffer(sim.model.nv);
  try {
    array(buffer.GetView()).fill(0);
    for (const load of loads)
      api.mj_applyFT(
        sim.model,
        sim.data,
        load.force.toArray(),
        load.torque.toArray(),
        load.point.toArray(),
        load.body,
        buffer,
      );
    return Float64Array.from(array(buffer.GetView()));
  } finally {
    buffer.delete();
  }
}
/** The unconstrained velocity response to a generalized impulse. */
export function solve(
  api: MainModule,
  sim: Simulation,
  force: Float64Array,
): Float64Array {
  if (force.length !== sim.model.nv)
    throw new Error("Generalized force size does not match model");
  const buffer = new api.DoubleBuffer(sim.model.nv);
  try {
    api.mj_solveM(sim.model, sim.data, buffer, Array.from(force));
    return Float64Array.from(array(buffer.GetView()));
  } finally {
    buffer.delete();
  }
}
export function inverseInertia(
  api: MainModule,
  sim: Simulation,
  direction: Float64Array,
): number {
  if (direction.length !== sim.model.nv)
    throw new Error("Generalized force size does not match model");
  // A load between immovable endpoints has no generalized direction.
  if (direction.every((value) => value === 0)) return 0;
  const response = solve(api, sim, direction);
  const inverse = direction.reduce(
    (sum, value, i) => sum + value * at(response, i),
    0,
  );
  if (!Number.isFinite(inverse) || inverse < 0)
    throw new Error("Invalid MuJoCo effective inverse inertia");
  return inverse;
}
export function apply(sim: Simulation, force: Float64Array, scale = 1): void {
  if (force.length !== sim.model.nv)
    throw new Error("Generalized force size does not match model");
  const target = array(sim.data.qfrc_applied);
  for (let i = 0; i < target.length; i++)
    target[i] = at(target, i) + at(force, i) * scale;
}
/**
 * Adds a world-space force and torque acting at `point`, the center of mass unless given, to the
 * coming step; an impulse changes the velocity at once instead.
 */
export function wrench(
  api: MainModule,
  sim: Simulation,
  id: number,
  force: Vector3,
  {
    torque = new Vector3(),
    point = vector(sim.data.xipos, id * 3),
    impulse = false,
  }: { torque?: Vector3; point?: Vector3; impulse?: boolean } = {},
): void {
  const generalized = project(api, sim, [{ body: id, force, torque, point }]);
  if (!impulse) {
    apply(sim, generalized);
    return;
  }
  const delta = solve(api, sim, generalized);
  const velocity = array(sim.data.qvel);
  for (let i = 0; i < velocity.length; i++)
    velocity[i] = at(velocity, i) + at(delta, i);
  api.mj_forward(sim.model, sim.data);
}

/** Applies each dynamic body's linear and angular damping, implicitly over the step. */
export function damp(api: MainModule, sim: Simulation, dt: number): void {
  const { model, data } = sim;
  for (const [body, id] of sim.bodies) {
    if (body.bodyType !== "dynamic") continue;
    if (!body.linearDamping && !body.angularDamping) continue;
    const v = velocity(api, sim, id);
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
    wrench(api, sim, id, force, { torque });
  }
}
