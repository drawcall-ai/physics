import { afterEach, expect, it } from "vitest";
import { Group, Matrix4, Quaternion, Vector3 } from "three";
import {
  BoxCollider,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { setupWorld, type RapierWorld } from "../src/index.js";

const worlds: RapierWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function createWorld() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  worlds.push(world);
  return world;
}
function inertialBody(world: RapierWorld) {
  return new RigidBody({
    world,
    colliders: false,
    mass: 1,
    diagonalInertia: [1, 1, 1],
  });
}

it("rejects invalid rays and accepts zero range and very small finite directions", async () => {
  const world = await createWorld();
  const body = new RigidBody({ world, type: "static", colliders: false });
  body.add(new BoxCollider());
  world.update(0);
  const origin = new Vector3(-2, 0, 0);
  for (const direction of [
    new Vector3(),
    new Vector3(NaN, 0, 0),
    new Vector3(Infinity, 0, 0),
  ])
    expect(() => world.raycast(origin, direction, 5)).toThrow();
  for (const distance of [-1, NaN, Infinity])
    expect(() =>
      world.raycast(origin, new Vector3(1, 0, 0), distance),
    ).toThrow();
  expect(() =>
    world.raycast(new Vector3(NaN, 0, 0), new Vector3(1, 0, 0), 5),
  ).toThrow();
  expect(world.raycast(origin, new Vector3(1, 0, 0), 0)).toBeNull();
  expect(
    world.raycast(origin, new Vector3(Number.MIN_VALUE, 0, 0), 5)?.distance,
  ).toBeCloseTo(1.5);
});

it("finds the closest surface after a step and excludes multiple bodies", async () => {
  const world = await createWorld();
  const bodies = [0, 3, 6].map((x) => {
    const body = new RigidBody({ world, colliders: false, mass: 1 });
    body.position.x = x;
    body.add(new BoxCollider());
    body.setVelocity({ linear: new Vector3(1, 0, 0) });
    return body;
  });
  const [first, second, third] = bodies;
  if (!first || !second || !third) throw new Error("Missing bodies");
  world.update(world.fixedDelta);
  const origin = new Vector3(-2, 0, 0),
    direction = new Vector3(2, 0, 0);
  const hit = world.raycast(origin, direction, 20);
  expect(hit?.body).toBe(first);
  expect(hit?.distance).toBeCloseTo(1.51, 5);
  expect(hit?.point.x).toBeCloseTo(-0.49, 5);
  expect(hit?.normal.toArray()).toEqual([-1, 0, 0]);
  const farther = world.raycast(origin, direction, 20, {
    excludeBodies: [first, second],
  });
  expect(farther?.body).toBe(third);
  expect(farther?.collider).toBe(third.children[0]);
  expect(farther?.distance).toBeCloseTo(7.51, 5);
});

it("applies effort through rotated local frames beneath a transformed parent", async () => {
  const world = await createWorld();
  const parent = new Group();
  parent.position.set(2, 3, 4);
  parent.rotation.z = 0.7;
  parent.scale.setScalar(2);
  const first = inertialBody(world),
    second = inertialBody(world);
  parent.add(first, second);
  const frame = new Matrix4().makeRotationY(Math.PI / 2);
  const hinge = new RevoluteJoint({
    body0: first,
    body1: second,
    axis: "Z",
    frame0: frame,
    frame1: frame,
  });
  hinge.setEffort(2);
  world.update(world.fixedDelta);
  const axis = new Vector3(1, 0, 0).applyQuaternion(parent.quaternion);
  expect(
    second.getVelocity().angular.distanceTo(axis.clone().multiplyScalar(0.02)),
  ).toBeLessThan(1e-6);
  expect(
    first.getVelocity().angular.distanceTo(axis.clone().multiplyScalar(-0.02)),
  ).toBeLessThan(1e-6);
  expect(hinge.getState().velocity).toBeCloseTo(0.04, 5);
});

it("applies opposing slider forces at offset anchors and clears pending effort on reset", async () => {
  const world = await createWorld();
  const first = inertialBody(world),
    second = inertialBody(world);
  const frame = new Matrix4().makeRotationY(Math.PI / 2).setPosition(0, 1, 0);
  const slider = new PrismaticJoint({
    body0: first,
    body1: second,
    axis: "X",
    frame0: frame,
    frame1: frame,
  });
  slider.setEffort(3);
  world.update(world.fixedDelta);
  expect(slider.getState().velocity).toBeGreaterThan(0);
  expect(
    first
      .getVelocity()
      .linear.clone()
      .add(second.getVelocity().linear)
      .length(),
  ).toBeLessThan(1e-6);
  expect(second.getVelocity().linear.z).toBeLessThan(0);
  slider.setEffort(100);
  world.reset();
  world.update(world.fixedDelta);
  expect(slider.getState().velocity).toBeCloseTo(0, 6);
  expect(first.getVelocity().angular.length()).toBeLessThan(1e-6);
  expect(second.getVelocity().angular.length()).toBeLessThan(1e-6);
});

it("requires sufficient dynamic colliderless inertia but accepts static and kinematic bodies", async () => {
  const world = await createWorld();
  const empty = new RigidBody({ world, colliders: false });
  expect(empty.getColliders()).toEqual([]);
  expect(() => world.update(0)).toThrow(/mass|inertia/i);
  empty.dispose();
  const massOnly = new RigidBody({ world, colliders: false, mass: 1 });
  expect(() => world.update(0)).toThrow(/mass|inertia/i);
  massOnly.dispose();
  new RigidBody({ world, type: "static", colliders: false });
  new RigidBody({ world, type: "kinematic", colliders: false });
  expect(() => world.update(0)).not.toThrow();
  expect(
    () => new RigidBody({ world, mass: 0, diagonalInertia: [1, 1, 1] }),
  ).toThrow();
  expect(
    () => new RigidBody({ world, mass: 1, diagonalInertia: [1, 1, 3] }),
  ).toThrow();
});

it("rotates the principal inertia axes used for angular response", async () => {
  const world = await createWorld();
  const axes = new Quaternion().setFromAxisAngle(
    new Vector3(0, 0, 1),
    Math.PI / 2,
  );
  const ordinary = new RigidBody({
    world,
    colliders: false,
    mass: 1,
    diagonalInertia: [1, 2, 3],
  });
  const rotated = new RigidBody({
    world,
    colliders: false,
    mass: 1,
    diagonalInertia: [1, 2, 3],
    principalAxes: [axes.x, axes.y, axes.z, axes.w],
  });
  world.update(0);
  for (const body of [ordinary, rotated])
    body.applyImpulse(new Vector3(0, 0, 1), new Vector3(0, 1, 0));
  expect(ordinary.getVelocity().angular.x).toBeCloseTo(1, 5);
  expect(rotated.getVelocity().angular.x).toBeCloseTo(0.5, 5);
  expect(rotated.getVelocity().angular.y).toBeCloseTo(0, 5);
});
