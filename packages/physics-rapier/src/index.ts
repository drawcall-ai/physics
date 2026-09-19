import { buildRegistered } from "@drawcall/physics";
import { RapierWorld, type RapierOptions } from "./world.js";
export type { RapierWorld, RapierOptions } from "./world.js";

export async function buildWorld(
  options: RapierOptions = {},
): Promise<RapierWorld> {
  return buildRegistered(async () => {
    const api = await import("@dimforge/rapier3d-compat");
    await api.init();
    const world = new RapierWorld(api, options);
    return world;
  });
}
