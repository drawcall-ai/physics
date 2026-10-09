import type * as Rapier from "@dimforge/rapier3d-compat";
import type {
  RaycastHit,
  RaycastOptions,
  RigidBody,
  Trigger,
} from "@drawcall/physics";
import { Vector3, type Object3D } from "three";

/** What a Rapier collider stands for in the scene, as a raycast hit names it. */
export type Owner = (
  { kind: "body"; body: RigidBody } | { kind: "trigger"; trigger: Trigger }
) & { object: Object3D };

export function raycast(
  api: typeof Rapier,
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
  origin: Vector3,
  direction: Vector3,
  maxDistance: number,
  options: RaycastOptions,
): RaycastHit | null {
  const groups = options.collisionGroups;
  const ray = new api.Ray(origin, direction);
  let closest: RaycastHit | null = null;
  // Per-collider casts see newly prepared and teleported colliders without a solver step.
  for (const [handle, owner] of owners) {
    if (
      owner.kind === "trigger"
        ? !options.includeTriggers
        : options.excludeBodies?.includes(owner.body)
    )
      continue;
    const collider = simulation.getCollider(handle);
    const mask = collider.collisionGroups();
    if (
      groups &&
      (((mask >>> 16) & groups.filter) === 0 ||
        (mask & 0xffff & groups.membership) === 0)
    )
      continue;
    const hit = collider.castRayAndGetNormal(
      ray,
      closest?.distance ?? maxDistance,
      false,
    );
    if (!hit) continue;
    closest = {
      ...owner,
      distance: hit.timeOfImpact,
      point: new Vector3()
        .copy(ray.dir)
        .multiplyScalar(hit.timeOfImpact)
        .add(origin),
      normal: new Vector3().copy(hit.normal),
    };
  }
  return closest;
}
