import type { DoubleBuffer, MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  type CapsuleCastHit,
  type CapsuleContact,
  type CapsuleQueryOptions,
  type CapsuleShape,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { array, at, vector } from "./heap.js";
import type { Simulation } from "./model/compile.js";
import { matches } from "./model/shapes.js";

const tolerance = 1e-6;
const padding = 1e-7;
const capsuleAxis = new Matrix4().makeRotationX(Math.PI / 2);

interface Candidate {
  id: number;
  body: RigidBody;
  object: CapsuleContact["object"];
}

function candidates(
  api: MainModule,
  sim: Simulation,
  options: CapsuleQueryOptions,
): Candidate[] {
  const result: Candidate[] = [];
  for (const [id, geometry] of sim.geometries) {
    const { owner: body, source: object, groups } = geometry;
    if (!(body instanceof RigidBody)) continue;
    if (options.excludeBodies?.includes(body)) continue;
    if (options.collisionGroups && !matches(options.collisionGroups, groups))
      continue;
    if (at(sim.model.geom_type, id) === api.mjtGeom.mjGEOM_HFIELD.value)
      throw new Error(
        "MuJoCo capsule queries do not support heightfields; use convex terrain colliders",
      );
    result.push({ id, body, object });
  }
  return result;
}

/** Temporary query geometry never changes body state, contacts, or the simulation clock. */
function withProbe<T>(
  api: MainModule,
  sim: Simulation,
  shape: CapsuleShape,
  pose: Matrix4,
  run: (probe: Probe) => T,
): T {
  const id = sim.capsuleProbe;
  const size = array(sim.model.geom_size).slice(id * 3, id * 3 + 3);
  const position = array(sim.data.geom_xpos).slice(id * 3, id * 3 + 3);
  const rotation = array(sim.data.geom_xmat).slice(id * 9, id * 9 + 9);
  const bound = at(sim.model.geom_rbound, id);
  const segment = new api.DoubleBuffer(6);
  try {
    const matrix = pose.clone().multiply(capsuleAxis).elements;
    array(sim.data.geom_xmat).set(
      [0, 1, 2].flatMap((row) =>
        [0, 1, 2].map((col) => {
          const value = matrix[col * 4 + row];
          if (value === undefined) throw new Error("Invalid capsule pose");
          return value;
        }),
      ),
      id * 9,
    );
    return run(new Probe(api, sim, shape, pose, segment));
  } finally {
    array(sim.model.geom_size).set(size, id * 3);
    array(sim.data.geom_xpos).set(position, id * 3);
    array(sim.data.geom_xmat).set(rotation, id * 9);
    array(sim.model.geom_rbound)[id] = bound;
    segment.delete();
  }
}

class Probe {
  readonly origin: Vector3;

  constructor(
    private readonly api: MainModule,
    private readonly sim: Simulation,
    private readonly shape: CapsuleShape,
    pose: Matrix4,
    private readonly segment: DoubleBuffer,
  ) {
    this.origin = new Vector3().setFromMatrixPosition(pose);
    this.move(this.origin);
  }

  move(position: Vector3): void {
    position.toArray(array(this.sim.data.geom_xpos), this.sim.capsuleProbe * 3);
  }

  contact(candidate: Candidate, maxDistance: number): CapsuleContact | null {
    const { api, sim, shape, segment } = this;
    // An inflated capsule gives a useful normal even at exact touching. The returned
    // distance is corrected back to the requested radius. A second inflation handles
    // the rare case where the first inflated surface itself touches exactly.
    for (const margin of [padding, padding * 2]) {
      array(sim.model.geom_size).set(
        [shape.radius + margin, shape.height / 2, 0],
        sim.capsuleProbe * 3,
      );
      array(sim.model.geom_rbound)[sim.capsuleProbe] =
        shape.radius + margin + shape.height / 2;
      const limit = maxDistance + margin;
      const distance = api.mj_geomDistance(
        sim.model,
        sim.data,
        candidate.id,
        sim.capsuleProbe,
        limit,
        segment,
      );
      if (!Number.isFinite(distance))
        throw new Error("MuJoCo capsule query returned a non-finite distance");
      if (distance >= limit) return null;
      const point = vector(segment.GetView(), 0);
      const normal = vector(segment.GetView(), 3).sub(point);
      if (normal.lengthSq() < 1e-24 || Math.abs(distance) < 1e-12) continue;
      normal.multiplyScalar(Math.sign(distance)).normalize();
      return {
        body: candidate.body,
        object: candidate.object,
        point,
        normal,
        distance: distance + margin,
      };
    }
    throw new Error("MuJoCo capsule query could not resolve a contact normal");
  }
}

export function capsuleContacts(
  api: MainModule,
  sim: Simulation,
  shape: CapsuleShape,
  pose: Matrix4,
  maxDistance: number,
  options: CapsuleQueryOptions,
): CapsuleContact[] {
  const obstacles = candidates(api, sim, options);
  return withProbe(api, sim, shape, pose, (probe) => {
    const contacts: CapsuleContact[] = [];
    for (const obstacle of obstacles) {
      const contact = probe.contact(obstacle, maxDistance + tolerance);
      if (contact && contact.distance <= maxDistance) contacts.push(contact);
    }
    return contacts;
  });
}

export function castCapsule(
  api: MainModule,
  sim: Simulation,
  shape: CapsuleShape,
  pose: Matrix4,
  displacement: Vector3,
  options: CapsuleQueryOptions,
): CapsuleCastHit | null {
  const length = displacement.length();
  if (length === 0) return null;
  const obstacles = candidates(api, sim, options);
  return withProbe(api, sim, shape, pose, (probe) => {
    let closest: CapsuleCastHit | null = null;
    for (const obstacle of obstacles) {
      const hit = cast(
        probe,
        obstacle,
        displacement,
        length,
        closest?.fraction ?? 1,
      );
      if (hit && (!closest || hit.fraction < closest.fraction)) closest = hit;
    }
    return closest;
  });
}

/** Conservative advancement against a fixed convex obstacle; no sampled teleport sweep. */
function cast(
  probe: Probe,
  obstacle: Candidate,
  displacement: Vector3,
  length: number,
  maxFraction: number,
): CapsuleCastHit | null {
  let fraction = 0;
  const position = new Vector3();
  for (let i = 0; i < 64; i++) {
    probe.move(
      position.copy(probe.origin).addScaledVector(displacement, fraction),
    );
    const contact = probe.contact(
      obstacle,
      length * (maxFraction - fraction) + tolerance,
    );
    if (!contact) return null;
    const closing = -displacement.dot(contact.normal);
    // A convex separating plane also proves that tangent and outward movement
    // cannot collide later, and lets an initially overlapping capsule escape.
    if (closing <= length * 1e-10) return null;
    if (contact.distance <= tolerance) return { ...contact, fraction };
    fraction += (contact.distance - tolerance * 0.5) / closing;
    if (fraction > maxFraction) return null;
  }
  throw new Error(
    "MuJoCo capsule sweep failed to converge after 64 iterations",
  );
}
