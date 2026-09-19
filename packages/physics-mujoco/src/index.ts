import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import { buildRegistered } from "@drawcall/physics";
import { MujocoWorld, type MujocoOptions } from "./world.js";
export type { MujocoWorld, MujocoOptions } from "./world.js";

import { Meshes } from "./model/meshes.js";

const modules = new Map<string | undefined, Promise<MainModule>>();
export async function buildWorld(
  options: MujocoOptions = {},
): Promise<MujocoWorld> {
  return buildRegistered(async (initial) => {
    const url = options.wasmUrl;
    let loading = modules.get(url);
    if (!loading) {
      loading = loadMujoco(url ? { locateFile: () => url } : undefined);
      modules.set(url, loading);
      loading.catch(() => modules.delete(url));
    }
    const meshes = new Meshes();
    const api = await loading;
    await meshes.prepare(initial, options.coacdWasmUrl);
    return new MujocoWorld(api, options, meshes);
  });
}
