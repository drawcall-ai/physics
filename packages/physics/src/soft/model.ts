import type { Vec3 } from "../colliders/collider.js";
import type { SoftMaterial } from "./material.js";

export interface ReducedRegion {
  readonly min: Vec3;
  readonly max: Vec3;
}

/**
 * Validated immutable mechanics supplied by a reducer. Producers must copy and freeze nested
 * data before sharing it between bodies. The basis does not certify approximation accuracy.
 */
export interface ReducedModel {
  readonly dofs: number;
  readonly material: SoftMaterial;
  /** FEM nodes, in metres in the undeformed component frame. */
  readonly positions: readonly number[];
  /** Contact triangles; each vertex must be a retained coordinate or a fixed mount node. */
  readonly triangles: readonly number[];
  readonly mount: readonly number[];
  /** Each retained node contributes three translation coordinates. */
  readonly coordinates: readonly number[];
  /** Row-major reconstruction from reduced displacement to FEM node displacement. */
  readonly basis: readonly number[];
  /** Row-major mass matrix: mount XYZ translation, then reduced coordinates. */
  readonly mass: readonly number[];
  readonly stiffness: readonly number[];
  /** Three columns mapping world acceleration into generalized force. */
  readonly gravity: readonly number[];
  readonly renderPositions: readonly number[];
  readonly renderTriangles: readonly number[];
  readonly renderBasis: readonly number[];
  /** Authored contact and mount regions in the undeformed component frame. */
  readonly contactRegion: ReducedRegion;
  readonly mountRegion: ReducedRegion;
  /** Maximum sampled vertex-to-surface distances in metres; not a continuous or mechanical error bound. */
  readonly surfaceError?: { readonly contact: number; readonly render: number };
  /** Volume weights and row-major displacement gradients (9 × dofs) per quadrature sample. */
  readonly quadrature?: {
    readonly weights: readonly number[];
    readonly gradients: readonly number[];
  };
}
