import type { Object3D } from "three";
import { rollback } from "./cleanup.js";
import { assertLive, type PhysicsWorld } from "./world.js";

const worlds = new Map<Object3D, PhysicsWorld>();
const building = new Set<Object3D>();

/** The world simulating `object`: the one rooted at its nearest ancestor that roots a world. */
export function worldOf(object: Object3D): PhysicsWorld | undefined {
  for (let node: Object3D | null = object; node; node = node.parent) {
    const world = worlds.get(node);
    if (world) return world;
  }
  return undefined;
}

export function requireWorld(object: Object3D): PhysicsWorld {
  const world = worldOf(object);
  if (!world)
    throw new Error(
      "Add the object under a built world's root for simulation commands or queries",
    );
  assertLive(world);
  return world;
}

export function assertOwned(world: PhysicsWorld, object: Object3D): void {
  assertLive(world);
  if (worldOf(object) !== world)
    throw new Error("Physics object is outside the world's root");
}

/** Reserves `root` while the backend loads, then publishes the world and prepares it without advancing time. */
export async function buildRooted<T extends PhysicsWorld>(
  root: Object3D,
  create: () => Promise<T>,
): Promise<T> {
  if (worlds.has(root) || building.has(root))
    throw new Error(
      "A physics world is already built or building for this root",
    );
  building.add(root);
  try {
    const world = await create();
    worlds.set(root, world);
    try {
      world.update(0);
    } catch (error) {
      worlds.delete(root);
      rollback(error, [() => world.dispose()], "Physics world build failed");
    }
    return world;
  } finally {
    building.delete(root);
  }
}

/** Backend integration: forgets a disposed world, leaving its root unsimulated. */
export function detach(world: PhysicsWorld): void {
  if (worlds.get(world.root) === world) worlds.delete(world.root);
}
