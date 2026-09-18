import type { DoubleBuffer, MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Trigger,
  ancestorBody,
  validateVector,
  type RaycastHit,
  type RaycastOptions,
} from "@drawcall/physics";
import { Vector3 } from "three";
import type { Compiled } from "./model/compile.js";
import { array, at } from "./values.js";
import { matches } from "./model/shapes.js";
import type { Interactions } from "@drawcall/physics";

export function sample(
  api: MainModule,
  compiled: Compiled,
  interactions: Interactions,
): void {
  const overlaps = new Map<Trigger, Set<RigidBody>>(),
    contacts = new Map<RigidBody, Set<RigidBody>>();
  const buffer = new api.DoubleBuffer(6);
  try {
    for (const [a, geom] of compiled.geometries) {
      if (!(geom.owner instanceof Trigger)) continue;
      const bodies = overlaps.get(geom.owner) ?? new Set<RigidBody>();
      overlaps.set(geom.owner, bodies);
      for (const [b, other] of compiled.geometries) {
        if (
          !(other.owner instanceof RigidBody) ||
          other.owner === ancestorBody(geom.owner) ||
          !matches(geom.groups, other.groups)
        )
          continue;
        if (
          api.mj_geomDistance(
            compiled.model,
            compiled.data,
            a,
            b,
            0.001,
            buffer,
          ) <= 0
        )
          bodies.add(other.owner);
      }
    }
  } finally {
    buffer.delete();
  }
  const values = compiled.data.contact;
  try {
    for (let i = 0; i < values.size(); i++) {
      const contact = values.get(i);
      if (!contact) throw new Error("Missing MuJoCo contact");
      try {
        if (contact.dist > 0) continue;
        const first = compiled.geometries.get(contact.geom1),
          second = compiled.geometries.get(contact.geom2);
        if (!first || !second)
          throw new Error("Contact references an unknown MuJoCo geometry");
        const a = first.owner,
          b = second.owner;
        if (!(a instanceof RigidBody) || !(b instanceof RigidBody) || a === b)
          continue;
        for (const [body, other] of [
          [a, b],
          [b, a],
        ] as const) {
          const set = contacts.get(body) ?? new Set<RigidBody>();
          set.add(other);
          contacts.set(body, set);
        }
      } finally {
        contact.delete();
      }
    }
  } finally {
    values.delete();
  }
  interactions.replace(overlaps, contacts);
}
export function raycast(
  api: MainModule,
  compiled: Compiled,
  origin: Vector3,
  direction: Vector3,
  maxDistance: number,
  options: RaycastOptions = {},
): RaycastHit | null {
  validateVector(origin);
  validateVector(direction);
  if (
    direction.lengthSq() === 0 ||
    !Number.isFinite(maxDistance) ||
    maxDistance < 0
  )
    throw new Error(
      "Raycast requires a nonzero direction and a finite nonnegative distance",
    );
  const unit = direction.clone().normalize();
  const normal = new api.DoubleBuffer(3);
  let result: RaycastHit | null = null;
  try {
    for (const [id, geom] of compiled.geometries) {
      if (geom.owner instanceof Trigger && !options.includeTriggers) continue;
      if (
        geom.owner instanceof RigidBody &&
        options.excludeBodies?.includes(geom.owner)
      )
        continue;
      if (
        options.collisionGroups &&
        !matches(options.collisionGroups, geom.groups)
      )
        continue;
      const distance = rayDistance(api, compiled, id, origin, unit, normal);
      if (
        distance < 0 ||
        distance > maxDistance ||
        (result && distance >= result.distance)
      )
        continue;
      const hit = {
        distance,
        point: origin.clone().addScaledVector(unit, distance),
        normal: new Vector3().fromArray(array(normal.GetView())),
        collider: geom.source,
      };
      result =
        geom.owner instanceof RigidBody
          ? { ...hit, kind: "body", body: geom.owner }
          : { ...hit, kind: "trigger", trigger: geom.owner };
    }
    return result;
  } finally {
    normal.delete();
  }
}

function rayDistance(
  api: MainModule,
  { model, data }: Compiled,
  id: number,
  origin: Vector3,
  direction: Vector3,
  normal: DoubleBuffer,
): number {
  const type = at(model.geom_type, id);
  const point = origin.toArray(),
    ray = direction.toArray();
  if (type === api.mjtGeom.mjGEOM_MESH.value)
    return api.mj_rayMesh(model, data, id, point, ray, normal);
  if (type === api.mjtGeom.mjGEOM_HFIELD.value)
    return api.mj_rayHfield(model, data, id, point, ray, normal);
  return api.mju_rayGeom(
    Array.from(array(data.geom_xpos).slice(id * 3, id * 3 + 3)),
    Array.from(array(data.geom_xmat).slice(id * 9, id * 9 + 9)),
    Array.from(array(model.geom_size).slice(id * 3, id * 3 + 3)),
    point,
    ray,
    type,
    normal,
  );
}
