import type { MainModule } from "@mujoco/mujoco";
import type { SoftBody } from "@drawcall/physics";
import { Triangle, Vector3 } from "three";
import { array, at, vector } from "../heap.js";
import type { Simulation } from "../model/compile.js";
import { name } from "../model/markup.js";
import type { ReducedMesh } from "./model.js";
import { reducedRenderPositions } from "./state.js";

interface Guard {
  body: SoftBody;
  id: number;
  positions: Float64Array;
}
const guards = new WeakMap<Simulation, Map<number, Guard>>();

/** Diagnostic contacts use MuJoCo's collision geometry but never its force constraints. */
export function guardXml(meshes: Iterable<ReducedMesh>): string {
  return Array.from(meshes, ({ body, model }) => {
    // Query-only flex. Use the moving mount body because world-parented rigid
    // flexes are culled against static geoms. Never enable this during a solve.
    return `<flex name="${name(body)}guard" dim="2" radius="0" body="${name(body)}r2" vertex="${model.renderPositions.join(" ")}" element="${model.renderTriangles.join(" ")}"><contact contype="0" conaffinity="0" selfcollide="none"/></flex>`;
  }).join("");
}

/** Runs a query-only collision pass, restoring physical contacts even when verification throws. */
export function checkReducedContacts(api: MainModule, sim: Simulation): void {
  if (!sim.reduced.size) return;
  let entries = guards.get(sim);
  if (!entries) {
    entries = new Map();
    for (const body of sim.reduced.keys()) {
      const id = api.mj_name2id(
        sim.model,
        api.mjtObj.mjOBJ_FLEX.value,
        `${name(body)}guard`,
      );
      if (id < 0) throw new Error("Missing reduced FEM collision guard");
      entries.set(id, { body, id, positions: new Float64Array() });
    }
    guards.set(sim, entries);
  }
  const point = new Vector3();
  for (const guard of entries.values()) {
    const state = sim.reduced.get(guard.body);
    if (!state) throw new Error("Missing reduced FEM collision state");
    const inverse = state.mesh.pose.clone().invert();
    const mount = new Vector3(
      ...state.joints
        .slice(0, 3)
        .map((joint) => at(sim.data.qpos, at(sim.model.jnt_qposadr, joint))),
    );
    guard.positions = reducedRenderPositions(sim, guard.body);
    const offset = at(sim.model.flex_vertadr, guard.id) * 3;
    const target = array(sim.model.flex_vert);
    for (let i = 0; i < guard.positions.length; i += 3)
      point
        .fromArray(guard.positions, i)
        .applyMatrix4(inverse)
        .sub(mount)
        .toArray(target, offset + i);
  }
  // These public pipeline stages update world poses and BVHs without assembling
  // contact constraints. A separate normal mj_step1/mj_forward follows the query.
  api.mj_kinematics(sim.model, sim.data);
  api.mj_comPos(sim.model, sim.data);
  api.mj_flex(sim.model, sim.data);
  try {
    for (const guard of entries.values()) {
      const physical = sim.reduced.get(guard.body);
      if (!physical) throw new Error("Missing reduced FEM collision state");
      array(sim.model.flex_contype)[guard.id] = at(
        sim.model.flex_contype,
        physical.id,
      );
      array(sim.model.flex_conaffinity)[guard.id] = at(
        sim.model.flex_conaffinity,
        physical.id,
      );
    }
    api.mj_collision(sim.model, sim.data);
    verify(sim);
  } finally {
    for (const guard of entries.values()) {
      array(sim.model.flex_contype)[guard.id] = 0;
      array(sim.model.flex_conaffinity)[guard.id] = 0;
    }
    api.mj_step1(sim.model, sim.data);
  }
}

/** Sampled intersections, not continuous collision detection or a self-contact guarantee. */
function verify(sim: Simulation): void {
  const entries = guards.get(sim);
  if (!entries?.size) return;
  const contacts = sim.data.contact;
  try {
    for (let i = 0; i < contacts.size(); i++) {
      const contact = contacts.get(i);
      if (!contact) throw new Error("Missing MuJoCo contact");
      try {
        for (let side = 0; side < 2; side++) {
          const guard = entries.get(at(contact.flex, side));
          if (!guard) continue;
          if (contact.efc_address >= 0)
            throw new Error(
              "Reduced FEM diagnostic contact unexpectedly generated a force constraint",
            );
          // Zero-thickness triangle/box intersections can have zero reported depth.
          if (contact.dist > 0) continue;
          const otherFlex = at(contact.flex, 1 - side);
          const state = sim.reduced.get(guard.body);
          if (!state) throw new Error("Missing reduced FEM collision state");
          if (
            entries.get(otherFlex)?.body === guard.body ||
            otherFlex === state.id
          )
            continue;
          const rest = restPoint(
            guard,
            at(contact.elem, side),
            at(contact.vert, side),
            vector(contact.pos, 0),
          );
          const inside = (region: {
            min: readonly number[];
            max: readonly number[];
          }) =>
            rest
              .toArray()
              .every(
                (value, axis) =>
                  value >= atValue(region.min, axis) - 1e-7 &&
                  value <= atValue(region.max, axis) + 1e-7,
              );
          if (inside(state.mesh.model.contactRegion)) continue;
          const other = sim.geometries.get(at(contact.geom, 1 - side))?.owner;
          if (
            other === state.mesh.target &&
            inside(state.mesh.model.mountRegion)
          )
            continue;
          throw new Error(
            `Reduced FEM contact outside the baked contact region (${guard.body.name || guard.body.uuid}); rebuild with a contact region covering this surface`,
          );
        }
      } finally {
        contact.delete();
      }
    }
  } finally {
    contacts.delete();
  }
}

function restPoint(
  guard: Guard,
  face: number,
  vertex: number,
  point: Vector3,
): Vector3 {
  const model = guard.body.model;
  if (!model) throw new Error("Missing reduced FEM model");
  if (vertex >= 0)
    return new Vector3().fromArray(model.renderPositions, vertex * 3);
  if (face < 0) throw new Error("Reduced FEM contact has no surface location");
  const indices = [0, 1, 2].map((corner) =>
    atValue(model.renderTriangles, face * 3 + corner),
  );
  const [a, b, c] = indices.map((index) =>
    new Vector3().fromArray(guard.positions, index * 3),
  );
  if (!a || !b || !c)
    throw new Error("Incomplete reduced FEM contact triangle");
  const triangle = new Triangle(a, b, c);
  const barycentric = triangle.getBarycoord(
    triangle.closestPointToPoint(point, new Vector3()),
    new Vector3(),
  );
  if (!barycentric) throw new Error("Degenerate reduced FEM contact triangle");
  return indices.reduce(
    (rest, index, corner) =>
      rest.addScaledVector(
        new Vector3().fromArray(model.renderPositions, index * 3),
        barycentric.getComponent(corner),
      ),
    new Vector3(),
  );
}

function atValue(values: readonly number[], index: number): number {
  const value = values[index];
  if (value === undefined) throw new Error("Invalid reduced FEM contact index");
  return value;
}
