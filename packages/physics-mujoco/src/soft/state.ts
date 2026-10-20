import type { MainModule } from "@mujoco/mujoco";
import type { SoftBody } from "@drawcall/physics";
import { updateSoftGeometry } from "@drawcall/physics/backend";
import { Vector3 } from "three";
import { array, at, vector } from "../heap.js";
import type { Simulation } from "../model/compile.js";
import { apply, project, solve, type Load } from "../force.js";
import type { SoftMesh } from "./mesh.js";

export interface SoftState {
  mesh: SoftMesh;
  id: number;
  nodes: number[];
}
export function softState(sim: Simulation, body: SoftBody): SoftState {
  const state = sim.soft.get(body);
  if (!state) throw new Error("Missing MuJoCo soft body");
  return state;
}
export function softPositions(sim: Simulation, body: SoftBody): Float32Array {
  const { id } = softState(sim, body);
  const start = at(sim.model.flex_vertadr, id) * 3;
  const count = at(sim.model.flex_vertnum, id) * 3;
  return Float32Array.from(
    array(sim.data.flexvert_xpos).slice(start, start + count),
  );
}
export function writeSoftGeometry(sim: Simulation): void {
  for (const [body, { mesh }] of sim.soft) {
    const positions = softPositions(sim, body);
    const render = new Float32Array(mesh.vertices.length * 3);
    for (const [i, node] of mesh.vertices.entries())
      render.set(positions.subarray(node * 3, node * 3 + 3), i * 3);
    updateSoftGeometry(body, render);
  }
}
export function carrySoft(
  previous: Simulation | undefined,
  next: Simulation,
): void {
  if (!previous) return;
  for (const [body, state] of next.soft) {
    const from = previous.soft.get(body);
    if (!from) continue;
    for (const [i, node] of state.nodes.entries()) {
      const old = from.nodes[i];
      if (old === undefined)
        throw new Error("Soft topology changed during simulation");
      const oldJoint = at(previous.model.body_jntadr, old);
      const joint = at(next.model.body_jntadr, node);
      const qpos = at(previous.model.jnt_qposadr, oldJoint);
      const qvel = at(previous.model.jnt_dofadr, oldJoint);
      array(next.data.qpos).set(
        array(previous.data.qpos).slice(qpos, qpos + 3),
        at(next.model.jnt_qposadr, joint),
      );
      array(next.data.qvel).set(
        array(previous.data.qvel).slice(qvel, qvel + 3),
        at(next.model.jnt_dofadr, joint),
      );
    }
  }
}
export function softLoad(
  api: MainModule,
  sim: Simulation,
  body: SoftBody,
  force: Vector3,
  point: Vector3 | undefined,
  impulse: boolean,
): void {
  const { nodes } = softState(sim, body);
  const loads: Load[] = [];
  if (point) {
    let distance = Infinity;
    let closest: number | undefined;
    for (const node of nodes) {
      const next = vector(sim.data.xpos, node * 3).distanceToSquared(point);
      if (next < distance) {
        distance = next;
        closest = node;
      }
    }
    if (closest === undefined)
      throw new Error("A soft load requires at least one node");
    loads.push({
      body: closest,
      force,
      torque: new Vector3(),
      point: vector(sim.data.xpos, closest * 3),
    });
  } else {
    const mass = nodes.reduce(
      (sum, node) => sum + at(sim.model.body_mass, node),
      0,
    );
    for (const node of nodes)
      loads.push({
        body: node,
        force: force
          .clone()
          .multiplyScalar(at(sim.model.body_mass, node) / mass),
        torque: new Vector3(),
        point: vector(sim.data.xpos, node * 3),
      });
  }
  const generalized = project(api, sim, loads);
  if (!impulse) return apply(sim, generalized);
  const delta = solve(api, sim, generalized);
  const velocity = array(sim.data.qvel);
  for (let i = 0; i < velocity.length; i++)
    velocity[i] = at(velocity, i) + at(delta, i);
  api.mj_forward(sim.model, sim.data);
}

/** Applies mass-proportional velocity drag, with the same step scaling as rigid-body damping. */
export function dampSoft(sim: Simulation, dt: number): void {
  const velocity = array(sim.data.qvel);
  const force = array(sim.data.qfrc_applied);
  for (const [body, state] of sim.soft) {
    const rate = body.linearDamping;
    if (!rate) continue;
    for (const node of state.nodes) {
      const start = at(sim.model.body_dofadr, node);
      const coefficient =
        (-at(sim.model.body_mass, node) * rate) / (1 + rate * dt);
      for (let axis = 0; axis < 3; axis++) {
        const dof = start + axis;
        force[dof] = at(force, dof) + coefficient * at(velocity, dof);
      }
    }
  }
}
