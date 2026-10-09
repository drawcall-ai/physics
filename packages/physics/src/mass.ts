import type { Vec3 } from "./colliders/collider.js";

export type MassProperties =
  | {
      readonly mass?: number;
      readonly centerOfMass?: never;
      readonly diagonalInertia?: never;
      readonly principalAxes?: never;
    }
  | {
      readonly mass: number;
      readonly centerOfMass: Vec3;
      readonly diagonalInertia: Vec3;
      readonly principalAxes?: readonly [number, number, number, number];
    };
export function validateMass(options: MassProperties): void {
  if (
    options.mass !== undefined &&
    (!Number.isFinite(options.mass) || options.mass <= 0)
  )
    throw new Error("Body mass must be positive");
  if (options.centerOfMass && !options.centerOfMass.every(Number.isFinite))
    throw new Error("Center of mass must be finite");
  const inertia = options.diagonalInertia;
  if (
    inertia &&
    (!inertia.every((v) => Number.isFinite(v) && v > 0) ||
      inertia.some((v) => 2 * v > inertia[0] + inertia[1] + inertia[2] + 1e-10))
  )
    throw new Error(
      "Principal inertia must be positive and satisfy the triangle inequality",
    );
  if (
    options.principalAxes &&
    (!options.principalAxes.every(Number.isFinite) ||
      Math.abs(Math.hypot(...options.principalAxes) - 1) > 1e-6)
  )
    throw new Error("Principal axes must be a normalized quaternion");
}
