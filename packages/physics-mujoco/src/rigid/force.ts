import type { MainModule } from "@mujoco/mujoco";
import { Vector3 } from "three";
import type { Simulation } from "../model/compile.js";
import { array, at, vector, quaternion } from "../heap.js";
import { project, solve, apply } from "../force.js";
import { velocity } from "./body.js";

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
