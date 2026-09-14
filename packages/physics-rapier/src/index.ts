import { setDefaultWorld } from "@drawcall/physics";
import { RapierWorld, type RapierOptions } from "./world.js";
export { RapierWorld, type RapierOptions } from "./world.js";

export async function setupWorld(
  options: RapierOptions = {},
): Promise<RapierWorld> {
  const api = await import("@dimforge/rapier3d-compat");
  await api.init();
  const world = new RapierWorld(api, options);
  setDefaultWorld(world);
  return world;
}
