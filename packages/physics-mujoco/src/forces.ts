import type { MainModule } from "@mujoco/mujoco";
import { Vector3 } from "three";
import type { Compiled } from "./model/compile.js";
import { array, at, vector, quaternion } from "./values.js";
import { velocity } from "./motion.js";

export interface Load {
  body: number;
  force: Vector3;
  torque: Vector3;
  point: Vector3;
}
/** Project world-space loads into generalized forces, including every body's lever arm. */
export function project(
  api: MainModule,
  compiled: Compiled,
  loads: readonly Load[],
): Float64Array {
  const buffer = new api.DoubleBuffer(compiled.model.nv);
  try {
    array(buffer.GetView()).fill(0);
    for (const load of loads)
      api.mj_applyFT(
        compiled.model,
        compiled.data,
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
  compiled: Compiled,
  force: Float64Array,
): Float64Array {
  if (force.length !== compiled.model.nv)
    throw new Error("Generalized force size does not match model");
  const buffer = new api.DoubleBuffer(compiled.model.nv);
  try {
    api.mj_solveM(compiled.model, compiled.data, buffer, Array.from(force));
    return Float64Array.from(array(buffer.GetView()));
  } finally {
    buffer.delete();
  }
}
export function inverseInertia(
  api: MainModule,
  compiled: Compiled,
  direction: Float64Array,
): number {
  if (direction.length !== compiled.model.nv)
    throw new Error("Generalized force size does not match model");
  // A load between immovable endpoints has no generalized direction.
  if (direction.every((value) => value === 0)) return 0;
  const response = solve(api, compiled, direction);
  const inverse = direction.reduce(
    (sum, value, i) => sum + value * at(response, i),
    0,
  );
  if (!Number.isFinite(inverse) || inverse < 0)
    throw new Error("Invalid MuJoCo effective inverse inertia");
  return inverse;
}
export function apply(
  compiled: Compiled,
  force: Float64Array,
  scale = 1,
): void {
  if (force.length !== compiled.model.nv)
    throw new Error("Generalized force size does not match model");
  const target = array(compiled.data.qfrc_applied);
  for (let i = 0; i < target.length; i++)
    target[i] = at(target, i) + at(force, i) * scale;
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
  const generalized = project(api, compiled, [
    { body: id, force, torque, point },
  ]);
  if (!impulse) {
    apply(compiled, generalized);
    return;
  }
  const delta = solve(api, compiled, generalized);
  const velocity = array(compiled.data.qvel);
  for (let i = 0; i < velocity.length; i++)
    velocity[i] = at(velocity, i) + at(delta, i);
  api.mj_forward(compiled.model, compiled.data);
}

export function applyBodyForces(
  api: MainModule,
  compiled: Compiled,
  dt: number,
): void {
  const { model, data } = compiled;
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
