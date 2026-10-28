import type { MainModule } from "@mujoco/mujoco";
import { build } from "@drawcall/physics/backend";
import { MujocoWorld } from "./world.js";
import type { MujocoWorldOptions } from "./options.js";
export type { MujocoWorld } from "./world.js";
export type { MujocoWorldOptions } from "./options.js";

const modules = new Map<string | undefined, Promise<MainModule>>();
async function load(url?: string): Promise<MainModule> {
  const { default: loadMujoco } = await import("../generated/mujoco.js");
  if (url) return loadMujoco({ locateFile: () => url });
  const { default: base64 } = await import("../generated/wasm.js");
  const binary = atob(base64);
  const wasmBinary = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) wasmBinary[i] = binary.charCodeAt(i);
  return loadMujoco({ wasmBinary });
}

/**
 * Builds the scene's rigid and deformable simulation. Loads embedded WASM lazily, running
 * single-threaded in Node.js or browsers without cross-origin isolation or an asset URL.
 * Webpack browser builds need `resolve: { fallback: { module: false } }` for the loader's
 * conditional Node import.
 *
 * Soft surfaces and explicit tetrahedral
 * volumes support Saint Venant–Kirchhoff elasticity; explicit volumes also support stable
 * Neo-Hookean elasticity. Curves use stretch springs without bending.
 * neoHookean selects MuJoCo's experimental simplified stable model; it remains defined
 * through element inversion, but does not prevent inversion.
 * Automatic volume meshing and worlds combining soft bodies with triggers are unsupported.
 * Soft attachments are compliant. Stiff, thin meshes may need smaller steps and more iterations.
 *
 * Reduced FEM uses the baked mass and material law with coupled rigid contact. Each reduced body
 * requires exactly one enabled attachment selecting its baked mount, attached to the world or
 * a static or translating kinematic body. Dynamic targets and rotating mounts are unsupported;
 * inverted elements and non-positive-definite elastic tangents are rejected.
 * Boxes use their exact convex hull in reduced scenes; capsule colliders are unsupported.
 * Native flex contacts distribute forces by inverse distance, not FEM shape functions;
 * refine the contact mesh and check convergence before relying on local contact deformation.
 * Self-contact is unsupported. Uncovered surface intersections stop the simulation;
 * this sampled diagnostic is not continuous collision detection.
 */
export async function buildWorld(
  options: MujocoWorldOptions,
): Promise<MujocoWorld> {
  const url = options.wasmUrl || undefined;
  let loading = modules.get(url);
  if (!loading) {
    loading = load(url);
    modules.set(url, loading);
    loading.catch(() => modules.delete(url));
  }
  return build(new MujocoWorld(await loading, options));
}
