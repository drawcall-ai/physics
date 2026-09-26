import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  buildRegistered,
  prepareConvexParts,
} from "@drawcall/physics";
import { heightfield } from "./model/heightfield.js";
import { MujocoWorld, type MujocoOptions } from "./world.js";
export type { MujocoWorld, MujocoOptions } from "./world.js";

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
    // MuJoCo collides only convex shapes: every triangle mesh but a static height grid
    // collides as convex parts, decomposed once here.
    const [api] = await Promise.all([
      loading,
      prepareConvexParts(
        initial,
        (body: RigidBody, geometry) =>
          body.bodyType !== "static" || !heightfield(geometry, "grid"),
      ),
    ]);
    return new MujocoWorld(api, options);
  });
}
