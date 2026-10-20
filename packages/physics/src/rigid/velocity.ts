import { Vector3 } from "three";
import type { Vec3 } from "../colliders/collider.js";
import { validateVector } from "../transforms.js";
import { assertBodyType, type RigidBody } from "./body.js";

export interface InitialVelocity {
  readonly linear: Vec3;
  readonly angular: Vec3;
}

export interface PhysicsVelocity {
  /** World space, m/s, of the center of mass. */
  linear: Vector3;
  /** World space, rad/s. */
  angular: Vector3;
}

const rest: InitialVelocity = { linear: [0, 0, 0], angular: [0, 0, 0] };

/** The velocity the body's options give it as it joins a world. */
export function initialVelocity(body: RigidBody): PhysicsVelocity {
  const { linear, angular } = body.options.velocity ?? rest;
  return { linear: new Vector3(...linear), angular: new Vector3(...angular) };
}

/** A frozen copy with both parts; rejects unknown keys and parts without three components. */
export function copyVelocity(value: Partial<InitialVelocity>): InitialVelocity {
  for (const key of Object.keys(value))
    if (key !== "linear" && key !== "angular")
      throw new Error(`Unknown rigid body velocity option: ${key}`);
  const copy: InitialVelocity = {
    linear: copyVec3(value.linear ?? rest.linear),
    angular: copyVec3(value.angular ?? rest.angular),
  };
  Object.freeze(copy);
  return copy;
}

function copyVec3(value: Vec3): Vec3 {
  if (value.length !== 3)
    throw new Error("Velocity parts must have three components");
  const copy: Vec3 = [value[0], value[1], value[2]];
  Object.freeze(copy);
  return copy;
}

export function validateVelocity(
  body: RigidBody,
  value: Partial<PhysicsVelocity>,
): void {
  if (value.linear) validateVector(value.linear);
  if (value.angular) validateVector(value.angular);
  assertBodyType(body, "dynamic", "Velocity");
}
