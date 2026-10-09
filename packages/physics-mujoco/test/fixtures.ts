import { afterEach } from "vitest";
import { BoxCollider, RigidBody, type RigidBodyType } from "@drawcall/physics";
import { BufferGeometry, Float32BufferAttribute, Scene } from "three";
import {
  buildWorld,
  type MujocoWorldOptions,
  type MujocoWorld,
} from "../src/index.js";

const worlds: MujocoWorld[] = [];
/** The scene worlds are built over by default; each test starts with a fresh one. */
export let scene = new Scene();
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
  resetScene();
});

/** Starts a fresh default scene, for a test that builds several worlds. */
export function resetScene(): void {
  scene = new Scene();
}

/** Builds a world without gravity at 0.01 s steps, disposed after the test. */
export async function createWorld(
  options: Partial<MujocoWorldOptions> = {},
): Promise<MujocoWorld> {
  const world = await buildWorld({
    scene,
    gravity: [0, 0, 0],
    fixedDelta: 0.01,
    ...options,
  });
  worlds.push(world);
  return world;
}

export function steps(world: MujocoWorld, count: number): void {
  for (let i = 0; i < count; i++) world.update(world.fixedDelta);
}

/** A unit box body of mass 2, added to the default scene. */
export function body(bodyType: RigidBodyType = "dynamic"): RigidBody {
  const body = new RigidBody({ bodyType, mass: 2 });
  body.add(new BoxCollider());
  scene.add(body);
  return body;
}

const vertices = new Float64Array([
  -0.48, -0.58, -0.25, 0.72, -0.58, -0.25, 0.72, 0.02, -0.25, 0.12, 0.02, -0.25,
  0.12, 0.82, -0.25, -0.48, 0.82, -0.25, -0.48, -0.58, 0.25, 0.72, -0.58, 0.25,
  0.72, 0.02, 0.25, 0.12, 0.02, 0.25, 0.12, 0.82, 0.25, -0.48, 0.82, 0.25,
]);
// prettier-ignore
const faces = new Int32Array([
  2, 1, 0,   5, 4, 3,   3, 2, 0,   0, 5, 3,   6, 7, 8,
  9, 10, 11, 6, 8, 9,   9, 11, 6,  7, 6, 1,   1, 6, 0,
  8, 7, 2,   2, 7, 1,   9, 8, 3,   3, 8, 2,   10, 9, 4,
  4, 9, 3,   6, 11, 0,  0, 11, 5,  11, 10, 5, 5, 10, 4,
]);

/** A closed, L-shaped prism whose notch a single convex hull would fill. */
export function concave(): BufferGeometry {
  return new BufferGeometry()
    .setAttribute("position", new Float32BufferAttribute(vertices, 3))
    .setIndex([...faces]);
}
