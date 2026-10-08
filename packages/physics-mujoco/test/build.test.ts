import { afterEach, expect, test } from "vitest";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  RevoluteJoint,
  Trigger,
  worldOf,
} from "@drawcall/physics";
import { Group, Object3D, Scene, Vector3 } from "three";
import { buildWorld as build, type MujocoWorld } from "../src/index.js";
import { concave } from "./mesh-fixture.js";

const worlds: MujocoWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function buildWorld(
  root: Object3D,
  options?: Parameters<typeof build>[1],
): Promise<MujocoWorld> {
  const world = await build(root, options);
  worlds.push(world);
  return world;
}

function mesh(geometry = concave()) {
  const body = new RigidBody({ type: "static" });
  body.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry),
  );
  return body;
}
const forward = new Vector3(0, 0, -1);

test("decomposes initial meshes and preserves concavities with ancestor scale", async () => {
  const geometry = concave();
  const body = mesh(geometry);
  const shared = mesh(geometry);
  shared.position.x = -5;
  const group = new Group().add(body);
  group.scale.setScalar(2);
  const root = new Scene().add(group, shared);
  const world = await buildWorld(root);
  expect(world.time).toBe(0);
  expect(world.raycast(new Vector3(1, 0.6, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(-0.5, 0.6, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  expect(world.raycast(new Vector3(-4.6, 0.3, 3), forward, 6)).toBeNull();
  // A later body reusing a mesh decomposed at build keeps its concavity.
  const later = mesh(geometry);
  later.position.x = 5;
  root.add(later);
  expect(world.raycast(new Vector3(5.4, 0.3, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(4.8, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body: later,
  });
  world.reset();
  expect(world.raycast(new Vector3(1, 0.6, 3), forward, 6)).toBeNull();
}, 15000);

test("building first leaves later trimeshes as a single hull", async () => {
  const root = new Scene();
  const world = await buildWorld(root);
  const body = mesh();
  root.add(body);
  expect(world.raycast(new Vector3(0.4, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
});

test("registrations during loading are included without losing removals", async () => {
  const removed = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const root = new Scene().add(removed);
  const pending = buildWorld(root);
  removed.removeFromParent();
  const body = new RigidBody({ mass: 1 }).add(new BoxCollider());
  body.position.y = 3;
  root.add(body);
  const world = await pending;
  expect(world.raycast(new Vector3(0, 0, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(0, 3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(3);
});

test("failed compilation preserves authoring objects for repair and retry", async () => {
  const body = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const joint = new RevoluteJoint({ body0: null, body1: body, limits: [0, 0] });
  const trigger = new Trigger().add(new BoxCollider());
  const root = new Scene().add(body, joint, trigger);
  await expect(buildWorld(root)).rejects.toThrow("nonzero");
  expect(worldOf(root)).toBeUndefined();
  expect(root.children).toEqual([body, joint, trigger]);
  joint.removeFromParent();
  const world = await buildWorld(root);
  world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(0);
});

test("rejects concurrent builds without disturbing the first", async () => {
  const root = new Scene();
  const pending = buildWorld(root);
  await expect(buildWorld(root)).rejects.toThrow("already built or building");
  const world = await pending;
  expect(worldOf(root)).toBe(world);
});

test("explicit convexHull remains a hull when present before building", async () => {
  const body = new RigidBody({ type: "static" }).add(
    new MeshCollider({ approximation: "convexHull" }).setGeometry(concave()),
  );
  const world = await buildWorld(new Scene().add(body));
  expect(world.raycast(new Vector3(0.4, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
});

test("changed initial geometry never reuses stale decompositions", async () => {
  const geometry = concave();
  const body = mesh(geometry);
  const world = await buildWorld(new Scene().add(body));
  expect(world.raycast(new Vector3(0.4, 0.3, 3), forward, 6)).toBeNull();
  geometry.translate(2, 0, 0);
  expect(world.raycast(new Vector3(2.4, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  expect(world.raycast(new Vector3(-0.3, 0.3, 3), forward, 6)).toBeNull();
});

test("collides a triangle mesh on a moving body as its convex parts", async () => {
  const body = new RigidBody();
  body.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(concave()),
  );
  const world = await buildWorld(new Scene().add(body), {
    gravity: [0, -9.81, 0],
  });
  // The notch of the L stays empty, which a single convex hull would fill.
  expect(world.raycast(new Vector3(0.4, 0.4, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(-0.2, 0.4, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  for (let i = 0; i < 50; i++) world.update(0.01);
  expect(body.position.y).toBeLessThan(-0.5);
}, 15000);
