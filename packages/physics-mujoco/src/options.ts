import type { PhysicsWorldOptions } from "@drawcall/physics";
import type { ModelOptions } from "./model/compile.js";

export interface MujocoWorldOptions extends PhysicsWorldOptions {
  /**
   * MuJoCo's friction cone. Elliptic cones model friction faithfully; the default pyramids are what
   * MuJoCo ships, and they proved more robust for kinematic contact at small steps in this build.
   */
  frictionCone?: "pyramidal" | "elliptic";
  /**
   * MuJoCo's impratio: how stiff friction constraints are relative to normal ones. At the default 1
   * a static grip still creeps, because soft friction trades slip for force; raising it converges on
   * Coulomb friction without raising the limit at which contacts start to slide. It is defined for
   * elliptic cones, so any value above 1 needs `frictionCone: "elliptic"`.
   */
  frictionImpedanceRatio?: number;
  /** Browser bundlers can pass an emitted asset URL; Node resolves the packaged WASM automatically. */
  wasmUrl?: string;
}

/** The model options the world's physics settings and these MuJoCo options give. */
export function modelOptions(
  options: MujocoWorldOptions,
  physics: Pick<ModelOptions, "fixedDelta" | "gravity" | "solverIterations">,
): ModelOptions {
  const frictionImpedanceRatio = options.frictionImpedanceRatio ?? 1;
  const frictionCone = options.frictionCone ?? "pyramidal";
  if (!Number.isFinite(frictionImpedanceRatio) || frictionImpedanceRatio < 1)
    throw new Error("frictionImpedanceRatio must be at least 1");
  if (frictionImpedanceRatio > 1 && frictionCone !== "elliptic")
    throw new Error(
      "frictionImpedanceRatio above 1 needs elliptic friction cones; set frictionCone",
    );
  return {
    ...physics,
    frictionImpedanceRatio,
    frictionCone,
  };
}
