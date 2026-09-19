import { Joint } from "./joint.js";
import { RigidBody } from "./body.js";
import type { Trigger } from "./trigger.js";
import { cleanup } from "./cleanup.js";
import { assertLive, type PhysicsWorld } from "./world.js";

type Registered = RigidBody | Joint | Trigger;
const objects = new Set<Registered>();
let world: PhysicsWorld | undefined;
let building = false;

/** Scene objects exist independently of the single attached simulation. */
export const registry = {
  get objects(): ReadonlySet<Registered> {
    return objects;
  },
  get world(): PhysicsWorld | undefined {
    return world;
  },
  register(object: Registered): void {
    if (object.disposed) throw new Error("Cannot register a disposed object");
    if (objects.has(object)) return;
    objects.add(object);
    try {
      world?.register(object);
    } catch (error) {
      objects.delete(object);
      throw error;
    }
  },
  unregister(object: Registered): void {
    if (!objects.delete(object)) return;
    const joints =
      object instanceof RigidBody
        ? [...objects].filter(
            (candidate) =>
              candidate instanceof Joint && candidate.connects(object),
          )
        : [];
    cleanup(
      [
        ...joints.map((joint) => () => joint.dispose()),
        () => world?.unregister(object),
      ],
      "Physics object removal failed",
    );
  },
  assertRegistered(object: Registered): void {
    if (object.disposed) throw new Error("Physics object has been disposed");
    if (!objects.has(object))
      throw new Error("Physics object is not registered");
  },
  requireWorld(object?: Registered): PhysicsWorld {
    if (object) this.assertRegistered(object);
    if (!world)
      throw new Error(
        "Call buildWorld() before simulation commands or queries",
      );
    assertLive(world);
    return world;
  },
  detach(value: PhysicsWorld): void {
    if (world === value) world = undefined;
  },
  clear(): void {
    cleanup(
      [...objects].map((object) => () => object.dispose()),
      "Physics registry cleanup failed",
    );
  },
};

export function assertOwned(value: PhysicsWorld, object: Registered): void {
  assertLive(value);
  registry.assertRegistered(object);
  if (world !== value)
    throw new Error("World is not attached to the physics registry");
}

/** Reserve the registry before asynchronous loading; publish only a prepared world. */
export async function buildRegistered<T extends PhysicsWorld>(
  create: (initial: readonly Registered[]) => Promise<T>,
): Promise<T> {
  if (world || building)
    throw new Error("A physics world is already built or building");
  building = true;
  let next: T | undefined;
  try {
    next = await create([...objects]);
    world = next;
    for (const object of objects) next.register(object);
    next.update(0);
    return next;
  } catch (error) {
    if (next) {
      const failed = next;
      world = undefined;
      cleanup(
        [
          // Native joints must be released before their endpoint bodies.
          ...[...objects]
            .sort(
              (a, b) => Number(b instanceof Joint) - Number(a instanceof Joint),
            )
            .map((object) => () => failed.unregister(object)),
          () => failed.dispose(),
          () => {
            throw error;
          },
        ],
        "Physics world build failed",
      );
    }
    throw error;
  } finally {
    building = false;
  }
}
