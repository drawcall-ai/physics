import type { Object3D } from "three";
import type { SoftMaterial, Vec3 } from "@drawcall/physics";

/** Included in every cache key. Bump when meshing, reduction or numerical dependencies change. */
export const REDUCER_VERSION = "fem-wasm@3";

export interface Bounds {
  readonly min: Vec3;
  readonly max: Vec3;
}

export interface FEMOptions {
  /** Density and elasticity are baked; changing either requires a new preparation. */
  readonly material: SoftMaterial;
  /** Fixed mounting region in scaled rest-local metres. The runtime attachment must cover these baked nodes. */
  readonly mount: Bounds;
  /** Surface region allowed to contact other bodies, in scaled rest-local metres. Uncovered intersections are rejected. */
  readonly contact: Bounds;
  readonly mesh?: {
    /** Target tetrahedral edge length in metres. Defaults to the largest extent / 8. */
    readonly cellSize?: number;
    /** Polynomial element order; defaults to quadratic. */
    readonly order?: 1 | 2;
    /**
     * Contact-skin coordinate budget including fixed vertices; defaults to 60.
     * Preparation rejects sampled contact-skin errors exceeding half cellSize; increase this budget on rejection.
     */
    readonly maxDofs?: number;
  };
}

/** Object requests must run inside GLTS so their source dependencies can be captured; the object remains caller-owned. */
export type FEMRequest = FEMOptions &
  (
    | { readonly url: string; readonly object?: never }
    | { readonly object: Object3D; readonly url?: never }
  );

export interface FEMInput extends FEMOptions {
  readonly positions: readonly number[];
  readonly triangles: readonly number[];
}
