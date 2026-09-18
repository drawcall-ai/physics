import { afterEach, expect, test, vi } from "vitest";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  JointDrive,
  RevoluteJoint,
} from "@drawcall/physics";
import { BoxGeometry, Vector3 } from "three";
import { setupWorld, type MujocoWorld } from "../src/index.js";

const worlds: MujocoWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function createWorld() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  worlds.push(world);
  return world;
}

test("unchanged mesh steps and target edits reuse collision geometry; raw vertex edits rebuild it", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  const geometry = new BoxGeometry();
  body.add(new MeshCollider().setGeometry(geometry));
  const clone = vi.spyOn(geometry, "clone");
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const drive = new JointDrive({ stiffness: 1, damping: 1 });
  joint.setDrive(drive);
  world.update(0);
  const initialCopies = clone.mock.calls.length;
  expect(initialCopies).toBeGreaterThan(0);
  for (let i = 0; i < 10; i++) {
    drive.setTarget({ position: i * 0.01 });
    world.update(world.fixedDelta);
  }
  expect(clone.mock.calls.length).toBe(initialCopies);
  const positions = geometry.getAttribute("position");
  for (let i = 0; i < positions.count; i++)
    positions.setX(i, positions.getX(i) * 3);
  world.update(0);
  expect(clone.mock.calls.length).toBeGreaterThan(initialCopies);
  const hit = world.raycast(new Vector3(3, 0, 0), new Vector3(-1, 0, 0), 5);
  expect(hit?.distance).toBeCloseTo(1.5, 2);
  geometry.dispose();
});

test("a failed rebuild keeps live state and retries after correction", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  const position = body.position.clone();
  body.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.8 });
  expect(() => world.update(world.fixedDelta)).toThrow(
    "equal static and dynamic friction",
  );
  expect(body.position.equals(position)).toBe(true);
  expect(body.getVelocity().linear.x).toBeCloseTo(2);
  expect(world.time).toBe(0.01);
  body.setMaterial({ staticFriction: 0.4, dynamicFriction: 0.4 });
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(0.04);
  expect(body.getVelocity().linear.x).toBeCloseTo(2);
});

test("failed initial compilation and queries do not capture reset poses or scale", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  body.position.x = 1;
  body.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.8 });
  expect(() => world.update(0)).toThrow("equal static and dynamic friction");
  body.setMaterial({ staticFriction: 0.4, dynamicFriction: 0.4 });
  world.raycast(new Vector3(3, 0, 0), new Vector3(-1, 0, 0), 5);
  body.position.x = 2;
  body.scale.setScalar(2);
  body.setVelocity({ linear: new Vector3(1, 0, 0) });
  world.update(world.fixedDelta);
  world.reset();
  expect(body.position.x).toBe(2);
  expect(body.getVelocity().linear.x).toBe(1);
});
