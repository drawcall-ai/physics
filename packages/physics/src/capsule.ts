import type { Matrix4, Object3D, Vector3 } from "three";
import { validateGroups, type CollisionGroups } from "./colliders/collider.js";
import type { RigidBody } from "./rigid/body.js";
import { assertRigidTransform } from "./transforms.js";

/** A capsule centered at its query pose, extending along local Y. Dimensions are in metres. */
export interface CapsuleShape {
  readonly radius: number;
  /** Length of the cylindrical part, excluding the caps. */
  readonly height: number;
}

/** Capsule queries intersect rigid colliders; triggers and soft bodies are excluded. */
export interface CapsuleQueryOptions {
  readonly collisionGroups?: CollisionGroups;
  readonly excludeBodies?: readonly RigidBody[];
}

export interface CapsuleContact {
  body: RigidBody;
  /** The explicit collider, or the visual mesh its automatic collider represents. */
  object: Object3D;
  /** Point on the obstacle, in world space. */
  point: Vector3;
  /** World-space unit normal pointing from the obstacle toward the capsule. */
  normal: Vector3;
  /** Signed separation in metres; negative means penetration. */
  distance: number;
}

export interface CapsuleCastHit extends CapsuleContact {
  /** Fraction of the requested translation, in [0, 1]. */
  fraction: number;
}

export function validateCapsule(
  shape: CapsuleShape,
  pose: Matrix4,
  options: CapsuleQueryOptions,
): void {
  if (
    !Number.isFinite(shape.radius) ||
    shape.radius <= 0 ||
    !Number.isFinite(shape.height) ||
    shape.height <= 0
  )
    throw new Error("Capsule dimensions must be positive and finite");
  assertRigidTransform(pose);
  if (options.collisionGroups) validateGroups(options.collisionGroups);
}
