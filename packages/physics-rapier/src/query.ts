import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  type RigidBody,
  type RaycastOptions,
  type RaycastHit,
  validateVector,
  validateGroups,
  resolveCollider,
  resolveCollisionGroups,
  splitTransform,
  type Trigger,
} from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import type { BodyBinding } from "./body.js";
import { descriptor } from "./shapes.js";

export function raycast(
  api: typeof Rapier,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
  origin: Vector3,
  direction: Vector3,
  maxDistance: number,
  options: RaycastOptions = {},
  triggers: Iterable<Trigger> = [],
): RaycastHit | null {
  validateVector(origin);
  validateVector(direction);
  const scale = Math.max(
    Math.abs(direction.x),
    Math.abs(direction.y),
    Math.abs(direction.z),
  );
  if (scale === 0 || !Number.isFinite(maxDistance) || maxDistance < 0)
    throw new Error(
      "Ray requires a nonzero direction and finite nonnegative distance",
    );
  const groups = options.collisionGroups;
  if (groups) validateGroups(groups);
  const ray = new api.Ray(
    origin,
    new Vector3(
      direction.x / scale,
      direction.y / scale,
      direction.z / scale,
    ).normalize(),
  );
  let closest: RaycastHit | null = null;
  // Per-collider casts include newly prepared and teleported bodies without a solver step.
  for (const [body, binding] of bodies) {
    if (options.excludeBodies?.includes(body)) continue;
    for (let i = 0; i < binding.body.numColliders(); i++) {
      const collider = binding.body.collider(i);
      if (collider.isSensor()) continue;
      const source = binding.sources.get(collider.handle);
      if (!source) throw new Error("Missing body collider source");
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
      closest = {
        kind: "body",
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
  if (!options.includeTriggers) return closest;
  for (const trigger of triggers) {
    for (const collider of trigger.getColliders()) {
      const mask = resolveCollisionGroups(collider, trigger);
      if (
        groups &&
        (!(mask.membership & groups.filter) ||
          !(mask.filter & groups.membership))
      )
        continue;
      const resolved = resolveCollider(trigger, collider);
      resolved.matrix.premultiply(splitTransform(trigger.matrixWorld).pose);
      const shape = descriptor(api, resolved.shape).shape;
      const hit = shape.castRayAndGetNormal(
        ray,
        new Vector3().setFromMatrixPosition(resolved.matrix),
        new Quaternion().setFromRotationMatrix(resolved.matrix),
        closest?.distance ?? maxDistance,
        false,
      );
      if (!hit) continue;
      closest = {
        kind: "trigger",
        trigger,
        collider,
        distance: hit.timeOfImpact,
        point: new Vector3()
          .copy(ray.dir)
          .multiplyScalar(hit.timeOfImpact)
          .add(origin),
        normal: new Vector3().copy(hit.normal),
      };
    }
  }
  return closest;
}
