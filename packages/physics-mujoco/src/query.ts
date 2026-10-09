import type { DoubleBuffer, MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Trigger,
  type RaycastHit,
  type RaycastOptions,
} from "@drawcall/physics";
import { Vector3 } from "three";
import type { Simulation } from "./model/compile.js";
import { array, at } from "./heap.js";
import { matches, type Geometry } from "./model/shapes.js";

/** MuJoCo filters rays by geometry group; the adapter filters by collider instead. */
const everyGroup = [1, 1, 1, 1, 1, 1];
export function raycast(
  api: MainModule,
  sim: Simulation,
  origin: Vector3,
  unit: Vector3,
  maxDistance: number,
  options: RaycastOptions,
): RaycastHit | null {
  const normal = new api.DoubleBuffer(3);
  const found = new api.IntBuffer(1);
  try {
    // One accelerated query beats intersecting every geometry from JavaScript.
    const distance = api.mj_ray(
      sim.model,
      sim.data,
      origin.toArray(),
      unit.toArray(),
      everyGroup,
      true,
      -1,
      found,
      normal,
    );
    if (distance < 0 || distance > maxDistance) return null;
    const geom = sim.geometries.get(at(found.GetView(), 0));
    if (geom && allows(geom, options))
      return hit(geom, origin, unit, distance, normal);
    // The closest geometry is filtered out, so the rest have to be intersected.
    return nearest(api, sim, origin, unit, maxDistance, options, normal);
  } finally {
    found.delete();
    normal.delete();
  }
}

function allows(geom: Geometry, options: RaycastOptions): boolean {
  if (geom.owner instanceof Trigger && !options.includeTriggers) return false;
  if (
    geom.owner instanceof RigidBody &&
    options.excludeBodies?.includes(geom.owner)
  )
    return false;
  return (
    !options.collisionGroups || matches(options.collisionGroups, geom.groups)
  );
}

function hit(
  geom: Geometry,
  origin: Vector3,
  unit: Vector3,
  distance: number,
  normal: DoubleBuffer,
): RaycastHit {
  const common = {
    distance,
    point: origin.clone().addScaledVector(unit, distance),
    normal: new Vector3().fromArray(array(normal.GetView())),
    object: geom.source,
  };
  return geom.owner instanceof RigidBody
    ? { ...common, kind: "body", body: geom.owner }
    : { ...common, kind: "trigger", trigger: geom.owner };
}

function nearest(
  api: MainModule,
  sim: Simulation,
  origin: Vector3,
  unit: Vector3,
  maxDistance: number,
  options: RaycastOptions,
  normal: DoubleBuffer,
): RaycastHit | null {
  let result: RaycastHit | null = null;
  for (const [id, geom] of sim.geometries) {
    if (!allows(geom, options)) continue;
    const distance = rayDistance(api, sim, id, origin, unit, normal);
    if (
      distance < 0 ||
      distance > maxDistance ||
      (result && distance >= result.distance)
    )
      continue;
    result = hit(geom, origin, unit, distance, normal);
  }
  return result;
}

function rayDistance(
  api: MainModule,
  { model, data }: Simulation,
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
