import type { Object3DEventMap } from "three";
import type { RigidBody } from "./rigid/body.js";
import type { SoftBody } from "./soft/body.js";

export type Body = RigidBody | SoftBody;

export interface BodyEventMap extends Object3DEventMap {
  contactbegin: { readonly otherBody: Body };
  contactend: { readonly otherBody: Body };
}

export function validateDamping(value: number): void {
  if (!Number.isFinite(value) || value < 0)
    throw new Error("Damping must be finite and nonnegative");
}
