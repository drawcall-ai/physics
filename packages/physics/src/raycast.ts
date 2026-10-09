import { Vector3, type Object3D } from "three";
import type { RigidBody } from "./body.js";
import { validateGroups, type CollisionGroups } from "./colliders/collider.js";
import type { Trigger } from "./trigger.js";
import { validateVector } from "./transforms.js";

export interface RaycastOptions {
  readonly collisionGroups?: CollisionGroups;
  readonly includeTriggers?: boolean;
  readonly excludeBodies?: readonly RigidBody[];
}
interface RaycastGeometry {
  distance: number;
  point: Vector3;
  normal: Vector3;
  /** The collider hit, or the visual mesh an automatic collider stands for. */
  object: Object3D;
}
export type RaycastHit = RaycastGeometry &
  ({ kind: "body"; body: RigidBody } | { kind: "trigger"; trigger: Trigger });

/** Validates a raycast's arguments and returns its unit direction. */
export function rayDirection(
  origin: Vector3,
  direction: Vector3,
  maxDistance: number,
  options: RaycastOptions,
): Vector3 {
  validateVector(origin);
  validateVector(direction);
  const scale = Math.max(
    Math.abs(direction.x),
    Math.abs(direction.y),
    Math.abs(direction.z),
  );
  if (scale === 0 || !Number.isFinite(maxDistance) || maxDistance < 0)
    throw new Error(
      "Raycast requires a nonzero direction and a finite nonnegative distance",
    );
  if (options.collisionGroups) validateGroups(options.collisionGroups);
  // Dividing each component keeps subnormal directions finite.
  return new Vector3(
    direction.x / scale,
    direction.y / scale,
    direction.z / scale,
  ).normalize();
}
