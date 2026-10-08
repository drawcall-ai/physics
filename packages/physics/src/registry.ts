import type { Object3D } from "three";
import { Joint } from "./joint.js";
import { RigidBody } from "./body.js";
import type { Trigger } from "./trigger.js";
import { cleanup, rollback } from "./cleanup.js";
import { assertLive, type PhysicsWorld } from "./world.js";

type Registered = RigidBody | Joint | Trigger;
const objects = new Set<Registered>();
let world: PhysicsWorld | undefined;
let building = false;

/** Live scene objects; the single attached world simulates those inside its root. */
export const registry = {
  get objects(): ReadonlySet<Registered> {
    return objects;
  },
  get world(): PhysicsWorld | undefined {
    return world;
  },
  /** The attached world when it simulates `object`. */
  worldOf(object: Registered): PhysicsWorld | undefined {
    return world && inside(world.root, object) ? world : undefined;
  },
  register(object: Registered): void {
    if (object.disposed) throw new Error("Cannot register a disposed object");
    objects.add(object);
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
    if (object) assertInside(world, object);
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

/** Whether a world simulating `root` includes the object: it sits under `root`, or a joint's bodies do. */
export function inside(root: Object3D, object: Registered): boolean {
  if (object instanceof Joint) {
    const { body0, body1 } = object.options;
    return (!body0 || inside(root, body0)) && inside(root, body1);
  }
  for (let node: Object3D | null = object; node; node = node.parent)
    if (node === root) return true;
  return false;
}

export function assertOwned(value: PhysicsWorld, object: Registered): void {
  assertLive(value);
  registry.assertRegistered(object);
  if (world !== value)
    throw new Error("World is not attached to the physics registry");
  assertInside(value, object);
}

function assertInside(value: PhysicsWorld, object: Registered): void {
  if (!inside(value.root, object))
    throw new Error("Physics object is outside the world's root");
}

/** Reserve the registry before asynchronous loading; publish only a prepared world. */
export async function buildRegistered<T extends PhysicsWorld>(
  root: Object3D,
  create: (initial: readonly Registered[]) => Promise<T>,
): Promise<T> {
  if (world || building)
    throw new Error("A physics world is already built or building");
  building = true;
  let next: T | undefined;
  try {
    next = await create([...objects].filter((object) => inside(root, object)));
    world = next;
    next.update(0);
    return next;
  } catch (error) {
    if (next) {
      const failed = next;
      world = undefined;
      rollback(
        error,
        [
          // Native joints must be released before their endpoint bodies.
          ...[...objects]
            .sort(
              (a, b) => Number(b instanceof Joint) - Number(a instanceof Joint),
            )
            .map((object) => () => failed.unregister(object)),
          () => failed.dispose(),
        ],
        "Physics world build failed",
      );
    }
    throw error;
  } finally {
    building = false;
  }
}
