import type { PhysicsWorldOptions } from "@drawcall/physics";
import { build } from "@drawcall/physics/backend";
import { RapierWorld } from "./world.js";
export type { RapierWorld } from "./world.js";

/** Builds the world that simulates the bodies, joints and triggers under `options.scene`. */
export async function buildWorld(
  options: PhysicsWorldOptions,
): Promise<RapierWorld> {
  const api = await import("@dimforge/rapier3d-compat");
  await api.init();
  return build(new RapierWorld(api, options));
}
