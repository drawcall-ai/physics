import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import { build } from "@drawcall/physics/backend";
import { MujocoWorld } from "./world.js";
import type { MujocoWorldOptions } from "./options.js";
export type { MujocoWorld } from "./world.js";
export type { MujocoWorldOptions } from "./options.js";

const modules = new Map<string | undefined, Promise<MainModule>>();
/**
 * Builds the scene's rigid and deformable simulation. Soft surfaces and explicit tetrahedral
 * volumes support Saint Venant–Kirchhoff elasticity; explicit volumes also support stable
 * Neo-Hookean elasticity. Curves use stretch springs without bending.
 * neoHookean selects MuJoCo's experimental simplified stable model; it remains defined
 * through element inversion, but does not prevent inversion.
 * Automatic volume meshing and worlds combining soft bodies with triggers are unsupported.
 * Soft scenes use MuJoCo's CG solver for coupled elastic attachments.
 */
export async function buildWorld(
  options: MujocoWorldOptions,
): Promise<MujocoWorld> {
  const url = options.wasmUrl;
  let loading = modules.get(url);
  if (!loading) {
    loading = loadMujoco(url ? { locateFile: () => url } : undefined);
    modules.set(url, loading);
    loading.catch(() => modules.delete(url));
  }
  return build(new MujocoWorld(await loading, options));
}
