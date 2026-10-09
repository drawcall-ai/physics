import type { Object3D } from "three";
import { RigidBody } from "./body.js";
import { Joint } from "./joints/joint.js";
import { Trigger } from "./trigger.js";
import { refresh, type PhysicsWorld } from "./world.js";

export type Member = RigidBody | Joint | Trigger;

/** The live world of each scene, so a command finds the world an object sits under. */
const scenes = new WeakMap<Object3D, PhysicsWorld>();
/** The world each joined object belongs to. */
const joined = new WeakMap<Member, PhysicsWorld>();

/** Reserves the world's scene, which holds at most one live world. */
export function claimScene(world: PhysicsWorld): void {
  if (scenes.has(world.scene))
    throw new Error(
      "The scene already has a physics world; dispose that world first",
    );
  scenes.set(world.scene, world);
}

/** Frees the world's scene for another world, if the world holds it. */
export function releaseScene(world: PhysicsWorld): void {
  if (scenes.get(world.scene) === world) scenes.delete(world.scene);
}

/** Records that `object` joined `world`, or left every world. */
export function setJoined(object: Member, world: PhysicsWorld | undefined) {
  if (world) joined.set(object, world);
  else joined.delete(object);
}

/** The world `object` has joined, while that world's scene still holds it. */
export function joinedWorld(object: Member): PhysicsWorld | undefined {
  const world = joined.get(object);
  return world && holds(world.scene, object) ? world : undefined;
}

/**
 * Has the world that still holds `object` release it once its scene no longer does, so `world`
 * can take it. Its listeners may meanwhile have `world` take it already.
 */
export function claim(object: Member, world: PhysicsWorld): void {
  const owner = joined.get(object);
  if (owner && owner !== world && !holds(owner.scene, object)) {
    try {
      refresh(owner);
    } catch (error) {
      throw new Error(
        `Releasing physics object ${object.name || object.type} from its previous world failed`,
        { cause: error },
      );
    }
  }
  const current = joined.get(object);
  if (current && current !== world)
    throw new Error(
      `Physics object ${object.name || object.type} is under the scenes of two worlds; it can belong to one`,
    );
}

/**
 * The objects that entered the scene, joints after their bodies, and the members that left it,
 * joints before their bodies. Throws for a joint whose bodies are not both under the scene.
 */
export function scan(
  scene: Object3D,
  members: ReadonlySet<Member>,
): { entered: Member[]; left: Member[] } {
  const current = new Set<Member>();
  const joints: Joint[] = [];
  scene.traverse((object) => {
    if (object instanceof Joint) joints.push(object);
    else if (object instanceof RigidBody || object instanceof Trigger)
      current.add(object);
  });
  for (const joint of joints) {
    const { body0, body1 } = joint.options;
    if ((body0 && !current.has(body0)) || !current.has(body1))
      throw new Error(
        `Joint ${joint.name || joint.type} connects a body outside the world's scene`,
      );
    current.add(joint);
  }
  return {
    entered: [...current].filter((object) => !members.has(object)),
    left: [...members]
      .filter((object) => !current.has(object))
      .sort((a, b) => Number(b instanceof Joint) - Number(a instanceof Joint)),
  };
}

/**
 * The world simulation commands on `object` go to: the world it joined, or else the built
 * world whose scene holds it, which the object then joins.
 */
export function commandWorld(object: Member): PhysicsWorld {
  const world = joinedWorld(object) ?? ancestorWorld(object);
  if (!world)
    throw new Error(
      `Physics object ${object.name || object.type} is not under a built world's scene; add it under one`,
    );
  return world;
}

function ancestorWorld(object: Object3D): PhysicsWorld | undefined {
  for (let node: Object3D | null = object; node; node = node.parent) {
    const world = scenes.get(node);
    if (world) return world;
  }
  return undefined;
}

function holds(scene: Object3D, object: Object3D): boolean {
  for (let node: Object3D | null = object; node; node = node.parent)
    if (node === scene) return true;
  return false;
}
