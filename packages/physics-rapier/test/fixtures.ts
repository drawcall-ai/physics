import { afterEach } from "vitest";
import { RigidBody, type RigidBodyOptions } from "@drawcall/physics";
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

export function inertialBody(options: RigidBodyOptions = {}) {
  return new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
    colliders: false,
    ...options,
  });
}
