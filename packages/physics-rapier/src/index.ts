import type { Object3D } from "three";
import { buildRooted, prepareConvexParts } from "@drawcall/physics";
import { RapierWorld, type RapierOptions } from "./world.js";
export type { RapierWorld, RapierOptions } from "./world.js";

/** Builds the world that simulates the bodies, joints and triggers under `root`. */
export async function buildWorld(
  root: Object3D,
  options: RapierOptions = {},
): Promise<RapierWorld> {
  return buildRooted(root, async () => {
    // Triangle meshes on moving bodies collide as convex parts, decomposed once here.
    const [api] = await Promise.all([
      import("@dimforge/rapier3d-compat").then(async (api) => {
        await api.init();
        return api;
      }),
      prepareConvexParts(root, (body) => body.bodyType !== "static"),
    ]);
    return new RapierWorld(api, root, options);
  });
}
