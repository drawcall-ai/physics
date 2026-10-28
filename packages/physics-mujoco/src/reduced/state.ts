import type { MainModule } from "@mujoco/mujoco";
import {
  splitTransform,
  type SoftBody,
  type RigidBody,
} from "@drawcall/physics";
import { updateReducedGeometry } from "@drawcall/physics/backend";
import { evaluateReduced } from "./energy.js";
import { Matrix4, Quaternion, Vector3 } from "three";
import { array, at } from "../heap.js";
import { apply, solve } from "../force.js";
import type { Simulation } from "../model/compile.js";
import type { ReducedMesh } from "./model.js";
import {
  cholesky,
  forces,
  multiply,
  positions,
  serial,
  value,
} from "./matrix.js";

export interface ReducedState {
  mesh: ReducedMesh;
  id: number;
  joints: number[];
  springs: number[];
}

function stateOf(sim: Simulation, body: SoftBody): ReducedState {
  const state = sim.reduced.get(body);
  if (!state) throw new Error("Missing MuJoCo reduced FEM body");
  return state;
}

function displacement(sim: Simulation, state: ReducedState): Float64Array {
  return positions(
    state.joints.map((joint) =>
      at(sim.data.qpos, at(sim.model.jnt_qposadr, joint)),
    ),
  );
}

function reconstruct(
  state: ReducedState,
  q: Float64Array,
  render: boolean,
): Float64Array {
  const model = state.mesh.model;
  const rest = render ? model.renderPositions : model.positions;
  const basis = render ? model.renderBasis : model.basis;
  const result = Float64Array.from(rest);
  for (let row = 0; row < rest.length; row++) {
    let offset = value(q, row % 3);
    for (let column = 0; column < model.dofs; column++)
      offset += value(basis, row * model.dofs + column) * value(q, column + 3);
    result[row] = value(result, row) + offset;
  }
  return result;
}

export function reducedPositions(
  sim: Simulation,
  body: SoftBody,
): Float32Array {
  const state = stateOf(sim, body);
  const result = Float32Array.from(
    reconstruct(state, displacement(sim, state), false),
  );
  const point = new Vector3();
  for (let i = 0; i < result.length; i += 3)
    point.fromArray(result, i).applyMatrix4(state.mesh.pose).toArray(result, i);
  return result;
}

export function reducedRenderPositions(
  sim: Simulation,
  body: SoftBody,
): Float64Array {
  const state = stateOf(sim, body);
  const result = reconstruct(state, displacement(sim, state), true);
  const point = new Vector3();
  for (let i = 0; i < result.length; i += 3)
    point.fromArray(result, i).applyMatrix4(state.mesh.pose).toArray(result, i);
  return result;
}

export function writeReducedGeometry(sim: Simulation): void {
  for (const [body, state] of sim.reduced)
    updateReducedGeometry(
      body,
      reconstruct(state, displacement(sim, state), true),
    );
}

export function carryReduced(
  previous: Simulation | undefined,
  next: Simulation,
): void {
  if (!previous) return;
  for (const [body, state] of next.reduced) {
    const old = previous.reduced.get(body);
    if (!old) continue;
    for (const [i, joint] of state.joints.entries()) {
      const from = old.joints[i];
      if (from === undefined)
        throw new Error("Reduced FEM coordinates changed during simulation");
      array(next.data.qpos)[at(next.model.jnt_qposadr, joint)] = at(
        previous.data.qpos,
        at(previous.model.jnt_qposadr, from),
      );
      array(next.data.qvel)[at(next.model.jnt_dofadr, joint)] = at(
        previous.data.qvel,
        at(previous.model.jnt_dofadr, from),
      );
    }
  }
}

/** Update the native implicit spring metric and retain the exact nonlinear elastic force. */
export function applyReduced(api: MainModule, sim: Simulation): void {
  const loads: { state: ReducedState; force: Float64Array }[] = [];
  let changed = false;
  for (const state of sim.reduced.values()) {
    const model = state.mesh.model;
    const q = displacement(sim, state).subarray(3);
    const load = new Float64Array(model.dofs + 3);
    if (model.material.elasticity.model !== "linear") {
      const response = evaluateReduced(model, q);
      const factor = cholesky(
        serial(response.stiffness, model.dofs, 0),
        model.dofs,
      );
      // The same factors give native stiffness-proportional damping C(q) = dampingTime * K(q).
      for (const [column, tendon] of state.springs.entries()) {
        const start = at(sim.model.tendon_adr, tendon);
        for (let row = 0; row < model.dofs; row++)
          array(sim.model.wrap_prm)[start + row] = value(
            factor,
            row * model.dofs + column,
          );
      }
      const correction = multiply(response.stiffness, q);
      for (let i = 0; i < model.dofs; i++)
        load[i + 3] = value(response.force, i) + value(correction, i);
      changed = true;
    }
    const gravity = new Vector3()
      .fromArray(array(sim.model.opt.gravity))
      .applyQuaternion(
        new Quaternion().setFromRotationMatrix(state.mesh.pose).invert(),
      );
    for (let row = 0; row < load.length; row++)
      for (let axis = 0; axis < 3; axis++)
        load[row] =
          value(load, row) +
          value(model.gravity, row * 3 + axis) * gravity.getComponent(axis);
    loads.push({ state, force: load });
  }
  // Tendon coefficients participate in both passive forces and the contact inertia.
  if (changed) api.mj_step1(sim.model, sim.data);
  for (const load of loads)
    apply(sim, generalized(sim, load.state, load.force));
}

function generalized(
  sim: Simulation,
  state: ReducedState,
  load: Float64Array,
): Float64Array {
  const projected = forces(load);
  const result = new Float64Array(sim.model.nv);
  for (const [i, joint] of state.joints.entries())
    result[at(sim.model.jnt_dofadr, joint)] = value(projected, i);
  return result;
}

export function reducedLoad(
  api: MainModule,
  sim: Simulation,
  body: SoftBody,
  force: Vector3,
  point: Vector3 | undefined,
  impulse: boolean,
): void {
  const state = stateOf(sim, body);
  const model = state.mesh.model;
  const local = force
    .clone()
    .applyQuaternion(
      new Quaternion().setFromRotationMatrix(state.mesh.pose).invert(),
    );
  const load = new Float64Array(model.dofs + 3);
  if (point) {
    const nodes = reducedPositions(sim, body);
    let nearest = 0,
      distance = Infinity;
    for (let node = 0; node < nodes.length / 3; node++) {
      const next = new Vector3()
        .fromArray(nodes, node * 3)
        .distanceToSquared(point);
      if (next < distance) {
        nearest = node;
        distance = next;
      }
    }
    local.toArray(load);
    for (let column = 0; column < model.dofs; column++)
      for (let axis = 0; axis < 3; axis++)
        load[column + 3] =
          value(load, column + 3) +
          value(model.basis, (nearest * 3 + axis) * model.dofs + column) *
            local.getComponent(axis);
  } else {
    const mass = value(model.mass, 0);
    for (let row = 0; row < load.length; row++)
      for (let axis = 0; axis < 3; axis++)
        load[row] =
          value(load, row) +
          (value(model.gravity, row * 3 + axis) * local.getComponent(axis)) /
            mass;
  }
  const projected = generalized(sim, state, load);
  if (!impulse) return apply(sim, projected);
  const delta = solve(api, sim, projected);
  for (let i = 0; i < sim.model.nv; i++)
    array(sim.data.qvel)[i] = at(sim.data.qvel, i) + value(delta, i);
  api.mj_forward(sim.model, sim.data);
}

export function checkReducedMount(
  mesh: ReducedMesh,
  target: RigidBody,
  matrix: Matrix4,
): void {
  if (mesh.target !== target || !mesh.mountRotation) return;
  const rotation = new Quaternion().setFromRotationMatrix(
    splitTransform(matrix).pose,
  );
  if (mesh.mountRotation.angleTo(rotation) > 1e-5)
    throw new Error("MuJoCo reduced FEM does not support rotating mounts");
}
