import { Vector3 } from "three";
import type { RigidBody } from "./body.js";
import { splitTransform } from "./transforms.js";
import type { PhysicsVelocity } from "./world.js";

/** Authored velocity is independent of immutable creation options and backend reset snapshots. */
const velocities = new WeakMap<RigidBody, PhysicsVelocity>();
export function authoredVelocity(object: RigidBody): PhysicsVelocity {
  const value = velocities.get(object);
  return {
    linear: value?.linear.clone() ?? new Vector3(),
    angular: value?.angular.clone() ?? new Vector3(),
  };
}
export function setAuthoredVelocity(
  object: RigidBody,
  value: Partial<PhysicsVelocity>,
): void {
  const next = authoredVelocity(object);
  if (value.linear) next.linear.copy(value.linear);
  if (value.angular) next.angular.copy(value.angular);
  velocities.set(object, next);
}

export function authoredVelocityAtPoint(
  body: RigidBody,
  point: Vector3,
): Vector3 {
  const { linear, angular } = body.getVelocity();
  if (angular.lengthSq() === 0) return linear;
  const center = body.options.centerOfMass;
  if (!center)
    throw new Error(
      "Authoring the anchor velocity of a rotating body requires explicit mass properties",
    );
  const worldCenter = new Vector3(...center).applyMatrix4(
    splitTransform(body.matrixWorld).pose,
  );
  return linear.add(angular.cross(point.clone().sub(worldCenter)));
}
