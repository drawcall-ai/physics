import type * as Rapier from "@dimforge/rapier3d-compat";
import type { RigidBody, RaycastOptions, RaycastHit } from "@drawcall/physics";
import { Vector3 } from "three";
import type { BodyBinding } from "./body.js";

export function raycast(
  api: typeof Rapier,
  world: Rapier.World,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
  origin: Vector3,
  direction: Vector3,
  maxDistance: number,
  options: RaycastOptions = {},
): RaycastHit | null {
  if (
    ![
      origin.x,
      origin.y,
      origin.z,
      direction.x,
      direction.y,
      direction.z,
      maxDistance,
    ].every(Number.isFinite) ||
    Math.max(
      Math.abs(direction.x),
      Math.abs(direction.y),
      Math.abs(direction.z),
    ) === 0 ||
    maxDistance < 0
  )
    throw new Error(
      "Ray requires a finite origin, nonzero finite direction and nonnegative finite distance",
    );
  const groups = options.collisionGroups;
  if (
    groups &&
    ![groups.membership, groups.filter].every(
      (value) => Number.isInteger(value) && value >= 0 && value <= 65535,
    )
  )
    throw new Error("Collision groups must be unsigned 16-bit masks");
  const scale = Math.max(
    Math.abs(direction.x),
    Math.abs(direction.y),
    Math.abs(direction.z),
  );
  const ray = new api.Ray(
    origin,
    new Vector3(
      direction.x / scale,
      direction.y / scale,
      direction.z / scale,
    ).normalize(),
  );
  world.propagateModifiedBodyPositionsToColliders();
  let closest: RaycastHit | null = null;
  // Per-collider casts include newly prepared and teleported bodies without a solver step.
  for (const [body, binding] of bodies) {
    if (options.excludeBodies?.includes(body)) continue;
    for (let i = 0; i < binding.body.numColliders(); i++) {
      const collider = binding.body.collider(i);
      if (collider.isSensor() && !options.includeSensors) continue;
      const mask = collider.collisionGroups();
      if (
        groups &&
        (((mask >>> 16) & groups.filter) === 0 ||
          (mask & 65535 & groups.membership) === 0)
      )
        continue;
      const hit = collider.castRayAndGetNormal(
        ray,
        closest?.distance ?? maxDistance,
        false,
      );
      if (!hit) continue;
      const source = binding.sources.get(collider.handle);
      if (!source) throw new Error("Missing source for physics collider");
      closest = {
        distance: hit.timeOfImpact,
        point: new Vector3()
          .copy(ray.dir)
          .multiplyScalar(hit.timeOfImpact)
          .add(origin),
        normal: new Vector3().copy(hit.normal),
        body,
        collider: source,
      };
    }
  }
  return closest;
}
