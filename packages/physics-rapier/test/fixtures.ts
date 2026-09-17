import { afterEach } from "vitest";
import { BoxGeometry, Mesh } from "three";
import {
  RigidBody,
  type PhysicsWorld,
  type RigidBodyOptions,
  type RigidBodyType,
} from "@drawcall/physics";
import {
  setupWorld,
  type RapierOptions,
  type RapierWorld,
} from "../src/index.js";

const worlds: RapierWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

export async function createWorld(options: RapierOptions = {}) {
  const world = await setupWorld({
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

export function box(type: RigidBodyType = "dynamic") {
  const body = new RigidBody({ mass: 1, type });
  body.add(new Mesh(new BoxGeometry(1, 1, 1)));
  return body;
}

export function steps(world: PhysicsWorld, count = 120) {
  for (let i = 0; i < count; i++) world.update(world.fixedDelta);
}
