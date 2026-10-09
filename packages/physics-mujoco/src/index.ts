import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import { RigidBody } from "@drawcall/physics";
import { build, prepareConvexParts } from "@drawcall/physics/backend";
import { heightfield } from "./model/heightfield.js";
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
  // MuJoCo collides only convex shapes: every triangle mesh but a static height grid
  // collides as convex parts, decomposed once here.
  const [api] = await Promise.all([
    loading,
    prepareConvexParts(
      options.scene,
      (body: RigidBody, geometry) =>
        body.bodyType !== "static" || !heightfield(geometry, "grid"),
    ),
  ]);
  return build(new MujocoWorld(api, options));
}
