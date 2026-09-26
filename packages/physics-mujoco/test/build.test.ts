import { afterEach, expect, test } from "vitest";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  RevoluteJoint,
  Trigger,
  registry,
} from "@drawcall/physics";
import { Group, Vector3 } from "three";
import { buildWorld } from "../src/index.js";
import { concave } from "./mesh-fixture.js";

afterEach(() => {
  registry.world?.dispose();
  registry.clear();
});

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
  const root = new Group().add(body);
  root.scale.setScalar(2);
  const world = await buildWorld();
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
  expect(world.raycast(new Vector3(5.4, 0.3, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(4.8, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body: later,
  });
  world.reset();
  expect(world.raycast(new Vector3(1, 0.6, 3), forward, 6)).toBeNull();
}, 15000);

test("building first leaves later trimeshes as a single hull", async () => {
  const world = await buildWorld();
  const body = mesh();
  expect(world.raycast(new Vector3(0.4, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
});

test("registrations during loading are included without losing removals", async () => {
  const removed = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const pending = buildWorld();
  removed.dispose();
  const body = new RigidBody({ mass: 1 }).add(new BoxCollider());
  body.position.y = 3;
  const world = await pending;
  expect(removed.disposed).toBe(true);
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
  await expect(buildWorld()).rejects.toThrow("nonzero");
  expect(registry.world).toBeUndefined();
  expect([...registry.objects]).toEqual([body, joint, trigger]);
  expect(joint.disposed).toBe(false);
  joint.dispose();
  const world = await buildWorld();
  world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(0);
});

test("rejects concurrent builds without disturbing the first", async () => {
  const pending = buildWorld();
  await expect(buildWorld()).rejects.toThrow("already built or building");
  const world = await pending;
  expect(registry.world).toBe(world);
});

test("explicit convexHull remains a hull when present before building", async () => {
  const body = new RigidBody({ type: "static" }).add(
    new MeshCollider({ approximation: "convexHull" }).setGeometry(concave()),
  );
  const world = await buildWorld();
  expect(world.raycast(new Vector3(0.4, 0.3, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
});

test("changed initial geometry never reuses stale decompositions", async () => {
  const geometry = concave();
  const body = mesh(geometry);
  const world = await buildWorld();
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
  const world = await buildWorld({ gravity: [0, -9.81, 0] });
  // The notch of the L stays empty, which a single convex hull would fill.
  expect(world.raycast(new Vector3(0.4, 0.4, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(-0.2, 0.4, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  for (let i = 0; i < 50; i++) world.update(0.01);
  expect(body.position.y).toBeLessThan(-0.5);
}, 15000);
