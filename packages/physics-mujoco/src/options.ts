import type { PhysicsWorldOptions } from "@drawcall/physics";
import type { ModelOptions } from "./model/compile.js";

/** Shared world settings plus MuJoCo configuration; omitted `solverIterations` uses 50. */
export interface MujocoWorldOptions extends PhysicsWorldOptions {
  /**
   * MuJoCo arena memory in bytes. Omit to use MuJoCo's allocation estimate.
   * Large deformable models can exceed the WASM limit with the estimate; an explicit arena
   * still has to fit the workload. For example, `128 * 1024 ** 2` reserves 128 MiB.
   */
  memory?: number;
  /**
   * MuJoCo's friction cone. Elliptic cones model friction faithfully; the default pyramids are what
   * MuJoCo ships, and they proved more robust for kinematic contact at small steps in this build.
   */
  frictionCone?: "pyramidal" | "elliptic";
  /**
   * MuJoCo's impratio: how stiff friction constraints are relative to normal ones. At the default 1
   * a static grip still creeps, because soft friction trades slip for force; raising it converges on
   * Coulomb friction without raising the limit at which contacts start to slide. It is defined for
   * elliptic cones, so any value above 1 needs `frictionCone: "elliptic"`. A value around 50
   * is a starting point for grasping; calibrate it against the actual load and contact geometry.
   */
  frictionImpedanceRatio?: number;
  /** Optional WASM URL override. Omit to use the embedded MuJoCo binary in Node and browsers. */
  wasmUrl?: string;
}

/** The model options the world's physics settings and these MuJoCo options give. */
export function modelOptions(
  options: MujocoWorldOptions,
  physics: Pick<ModelOptions, "fixedDelta" | "gravity" | "solverIterations">,
): ModelOptions {
  const frictionImpedanceRatio = options.frictionImpedanceRatio ?? 1;
  const frictionCone = options.frictionCone ?? "pyramidal";
  if (
    options.memory !== undefined &&
    (!Number.isSafeInteger(options.memory) || options.memory <= 0)
  )
    throw new Error("memory must be a positive integer number of bytes");
  if (!Number.isFinite(frictionImpedanceRatio) || frictionImpedanceRatio < 1)
    throw new Error("frictionImpedanceRatio must be at least 1");
  if (frictionImpedanceRatio > 1 && frictionCone !== "elliptic")
    throw new Error(
      "frictionImpedanceRatio above 1 needs elliptic friction cones; set frictionCone",
    );
  return {
    ...physics,
    memory: options.memory,
    frictionImpedanceRatio,
    frictionCone,
  };
}
