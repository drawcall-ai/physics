import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import { build } from "@drawcall/physics/backend";
import { MujocoWorld } from "./world.js";
import type { MujocoWorldOptions } from "./options.js";
export type { MujocoWorld } from "./world.js";
export type { MujocoWorldOptions } from "./options.js";

const modules = new Map<string | undefined, Promise<MainModule>>();
/** Builds the world that simulates the bodies, joints and triggers under `options.scene`. */
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
