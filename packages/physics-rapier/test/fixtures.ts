import { afterEach } from "vitest";
import { BoxGeometry, Mesh, Scene } from "three";
import {
  RigidBody,
  type PhysicsWorld,
  type PhysicsWorldOptions,
  type RigidBodyOptions,
  type RigidBodyType,
} from "@drawcall/physics";
import { buildWorld, type RapierWorld } from "../src/index.js";

const worlds: RapierWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

/** Builds a world over a fresh scene unless given one; tests add what it simulates to `world.scene`. */
export async function createWorld(options: Partial<PhysicsWorldOptions> = {}) {
  const world = await buildWorld({
    scene: new Scene(),
    gravity: [0, 0, 0],
    fixedDelta: 0.01,
    ...options,
  });
  worlds.push(world);
  return world;
}

export function inertialBody(options: Partial<RigidBodyOptions> = {}) {
  return new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
    colliders: false,
    ...options,
  });
}

export const earth = { gravity: [0, -9.81, 0], fixedDelta: 1 / 60 } as const;

export function box(bodyType: RigidBodyType = "dynamic") {
  const body = new RigidBody({ mass: 1, bodyType });
  body.add(new Mesh(new BoxGeometry(1, 1, 1)));
  return body;
}

export function steps(world: PhysicsWorld, count = 120) {
  for (let i = 0; i < count; i++) world.update(world.fixedDelta);
}
