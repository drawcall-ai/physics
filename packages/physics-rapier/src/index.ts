import { buildRegistered, prepareConvexParts } from "@drawcall/physics";
import { RapierWorld, type RapierOptions } from "./world.js";
export type { RapierWorld, RapierOptions } from "./world.js";

export async function buildWorld(
  options: RapierOptions = {},
): Promise<RapierWorld> {
  return buildRegistered(async (initial) => {
    // Triangle meshes on moving bodies collide as convex parts, decomposed once here.
    const [api] = await Promise.all([
      import("@dimforge/rapier3d-compat").then(async (api) => {
        await api.init();
        return api;
      }),
      prepareConvexParts(initial, (body) => body.bodyType !== "static"),
    ]);
    const world = new RapierWorld(api, options);
    return world;
  });
}
