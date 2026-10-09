import type { Object3D, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import type { CollisionGroups } from "./colliders/collider.js";
import type { Trigger } from "./trigger.js";

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
