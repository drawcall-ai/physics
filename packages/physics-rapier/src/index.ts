import type { Object3D } from "three";
import { buildRegistered, prepareConvexParts } from "@drawcall/physics";
import { RapierWorld, type RapierOptions } from "./world.js";
export type { RapierWorld, RapierOptions } from "./world.js";

/** Builds the world that simulates the bodies and triggers under `root`, and the joints between them. */
export async function buildWorld(
  root: Object3D,
  options: RapierOptions = {},
): Promise<RapierWorld> {
  return buildRegistered(root, async (initial) => {
    // Triangle meshes on moving bodies collide as convex parts, decomposed once here.
    const [api] = await Promise.all([
      import("@dimforge/rapier3d-compat").then(async (api) => {
        await api.init();
        return api;
      }),
      prepareConvexParts(initial, (body) => body.bodyType !== "static"),
    ]);
    return new RapierWorld(api, root, options);
  });
}
