import type { PhysicsWorldOptions } from "@drawcall/physics";
import { build } from "@drawcall/physics/backend";
import { RapierWorld } from "./world.js";
export type { RapierWorld } from "./world.js";

/**
 * Builds the world that simulates the physics objects under `options.scene`.
 * Omitted `solverIterations` preserves the engine's default. Triangle meshes stay exact on
 * static bodies; moving meshes use convex decomposition. Static and dynamic friction must
 * match; torsional and rolling friction are unsupported.
 *
 * Soft surfaces and curves use spring elasticity; volumes use corotational or neo-Hookean
 * elasticity, with either explicit tetrahedra or automatic meshing of a closed surface.
 * Spring frequencies describe individual constraints and depend on mesh resolution;
 * they are not continuum cloth/rod moduli. Elastic volumes accept Young's modulus in Pa.
 * Density is integrated over segments, triangles, or tetrahedra and lumped onto their nodes.
 * Automatically meshed volumes distribute the authored total mass over their enclosing cage;
 * explicit tetrahedra follow the authored volume directly. Linear damping is a rate in s⁻¹.
 */
export async function buildWorld(
  options: PhysicsWorldOptions,
): Promise<RapierWorld> {
  const api = await import("@dimforge/rapier3d-compat");
  await api.init();
  return build(new RapierWorld(api, options));
}
