import {
  validateMaterial,
  type PhysicsMaterial,
} from "../colliders/collider.js";

export interface Softness {
  /**
   * Natural frequency in Hz of an individual spring constraint, not of the whole body.
   * Changing mesh resolution changes bulk stiffness; this is not Young's modulus.
   */
  readonly frequency: number;
  /** Dimensionless damping of this constraint's deformation, not overall body motion. */
  readonly dampingRatio: number;
}

export type Elasticity =
  | {
      /** Infinitesimal strain and rotation about the rest configuration. */
      readonly model: "linear";
      /** Pa. */
      readonly youngModulus: number;
      readonly poissonRatio: number;
      /** Stiffness-proportional damping, seconds. */
      readonly dampingTime?: number;
    }
  | {
      readonly model: "corotational" | "neoHookean";
      /** Pa. */
      readonly youngModulus: number;
      readonly poissonRatio: number;
      /** Dimensionless internal elastic damping, not damping of bulk motion. */
      readonly dampingRatio?: number;
      /** Stiffness-proportional damping in seconds; native MuJoCo neoHookean or reduced FEM. Mutually exclusive with dampingRatio. */
      readonly dampingTime?: number;
    }
  | {
      /**
       * Suitable for small strains with large rotations; not a general large-strain rubber law.
       * MuJoCo surfaces use 3D Lamé coefficients with zero through-thickness strain,
       * rather than a plane-stress or woven-fabric model.
       */
      readonly model: "saintVenantKirchhoff";
      readonly youngModulus: number;
      readonly poissonRatio: number;
      /** Stiffness-proportional Rayleigh damping, seconds. */
      readonly dampingTime?: number;
    }
  | {
      readonly model: "spring";
      readonly stretch: Softness;
      readonly bend?: Softness;
    };

/**
 * SI units. The selected backend must support the requested elasticity model.
 * One supplied friction coefficient supplies both; when omitted, both default to 0.5.
 */
export interface SoftMaterial extends PhysicsMaterial {
  /** Homogeneous rest density in kg/m³; surfaces use area × thickness and curves use cross-section × length. */
  readonly density: number;
  readonly elasticity: Elasticity;
}

export function copySoftMaterial(value: SoftMaterial): SoftMaterial {
  const { elasticity, ...contact } = value;
  keys(
    contact,
    [
      "density",
      "staticFriction",
      "dynamicFriction",
      "restitution",
      "torsionalFriction",
      "rollingFriction",
    ],
    "soft material",
  );
  validateMaterial(contact);
  if (!Number.isFinite(value.density) || value.density <= 0)
    throw new Error("Soft-body density must be positive and finite");
  if (elasticity.model === "spring") {
    keys(elasticity, ["model", "stretch", "bend"], "spring elasticity");
    softness(elasticity.stretch);
    if (elasticity.bend) softness(elasticity.bend);
    return Object.freeze({
      ...contact,
      elasticity: Object.freeze({
        ...elasticity,
        stretch: Object.freeze({ ...elasticity.stretch }),
        bend: elasticity.bend && Object.freeze({ ...elasticity.bend }),
      }),
    });
  }
  if (
    !["linear", "corotational", "neoHookean", "saintVenantKirchhoff"].includes(
      elasticity.model,
    )
  )
    throw new Error("Unknown soft-body elasticity model");
  keys(
    elasticity,
    [
      "model",
      "youngModulus",
      "poissonRatio",
      "dampingTime",
      ...(["linear", "saintVenantKirchhoff"].includes(elasticity.model)
        ? []
        : ["dampingRatio"]),
    ],
    "solid elasticity",
  );
  if (!Number.isFinite(elasticity.youngModulus) || elasticity.youngModulus <= 0)
    throw new Error("Young's modulus must be positive and finite");
  if (
    !Number.isFinite(elasticity.poissonRatio) ||
    elasticity.poissonRatio < 0 ||
    elasticity.poissonRatio >= 0.5
  )
    throw new Error("Poisson ratio must be in [0, 0.5)");
  const ratio =
    elasticity.model === "saintVenantKirchhoff" || elasticity.model === "linear"
      ? undefined
      : elasticity.dampingRatio;
  if (ratio !== undefined && elasticity.dampingTime !== undefined)
    throw new Error("Specify either dampingRatio or dampingTime, not both");
  for (const damping of [ratio, elasticity.dampingTime])
    if (damping !== undefined && (!Number.isFinite(damping) || damping < 0))
      throw new Error("Elastic damping must be finite and nonnegative");
  return Object.freeze({
    ...contact,
    elasticity: Object.freeze({ ...elasticity }),
  });
}

function softness(value: Softness): void {
  keys(value, ["frequency", "dampingRatio"], "spring softness");
  if (
    !Number.isFinite(value.frequency) ||
    value.frequency < 0 ||
    !Number.isFinite(value.dampingRatio) ||
    value.dampingRatio < 0
  )
    throw new Error(
      "Spring frequency and damping ratio must be finite and nonnegative",
    );
}

function keys(value: object, allowed: readonly string[], label: string): void {
  for (const key of Object.keys(value))
    if (!allowed.includes(key))
      throw new Error(`Unknown ${label} property: ${key}`);
}
