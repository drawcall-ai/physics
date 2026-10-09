import { build, prepareConvexParts } from "@drawcall/physics/backend";
import { RapierWorld, type RapierWorldOptions } from "./world.js";
export type { RapierWorld, RapierWorldOptions } from "./world.js";

/** Builds the world that simulates the bodies, joints and triggers under `options.scene`. */
export async function buildWorld(
  options: RapierWorldOptions,
): Promise<RapierWorld> {
  // Triangle meshes on moving bodies collide as convex parts, decomposed once here.
  const [api] = await Promise.all([
    import("@dimforge/rapier3d-compat").then(async (api) => {
      await api.init();
      return api;
    }),
    prepareConvexParts(options.scene, (body) => body.bodyType !== "static"),
  ]);
  return build(new RapierWorld(api, options));
}
