import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import { setDefaultWorld } from "@drawcall/physics";
import { MujocoWorld, type MujocoOptions } from "./world.js";
export { MujocoWorld, type MujocoOptions } from "./world.js";

const modules = new Map<string | undefined, Promise<MainModule>>();
export async function setupWorld(
  options: MujocoOptions = {},
): Promise<MujocoWorld> {
  const url = options.wasmUrl;
  let loading = modules.get(url);
  if (!loading) {
    loading = loadMujoco(url ? { locateFile: () => url } : undefined);
    modules.set(url, loading);
    loading.catch(() => modules.delete(url));
  }
  const world = new MujocoWorld(await loading, options);
  setDefaultWorld(world);
  return world;
}
