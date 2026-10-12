import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh, Vector3 } from "three";
import {
  DistanceJoint,
  FixedJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { createWorld, box, earth } from "./fixtures.js";

it("adds bodies after stepping without resetting existing velocities or poses", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const first = box();
  world.scene.add(first);
  first.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  const previous = first.position.x;
  const second = box();
  second.position.y = 4;
  world.scene.add(second);
  world.update(world.fixedDelta);
  expect(first.position.x).toBeGreaterThan(previous);
  expect(first.getVelocity().linear.x).toBeCloseTo(2);
  expect(second.position.y).toBeCloseTo(4);
});

it("adds joints after bodies are already simulating", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = 4;
  world.scene.add(body);
  world.update(world.fixedDelta);
  const anchor = body.position.y;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = anchor;
  world.scene.add(joint);
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(anchor, 2);
  joint.removeFromParent();
  for (let i = 0; i < 30; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(anchor - 0.5);
});

it("updates collider geometry without resetting the body", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const floor = new RigidBody({ bodyType: "static" });
  const mesh = new Mesh(new BoxGeometry(1, 1, 1));
  floor.add(mesh);
  const body = box();
  body.position.x = 3;
  world.scene.add(floor, body);
  world.update(world.fixedDelta);
  mesh.geometry = new BoxGeometry(8, 1, 1);
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(Math.abs(body.position.y)).toBeGreaterThan(0.8);
});

it("adds bodies created by before-step callbacks", async () => {
  const world = await createWorld(earth);
  let body: RigidBody | undefined;
  const stop = world.onBeforeStep(() => {
    body = box();
    body.position.y = 2;
    world.scene.add(body);
    stop();
  });
  world.update(world.fixedDelta);
  expect(body?.position.y).toBeLessThan(2);
});

it("removes automatic colliders when geometry is removed", async () => {
  const world = await createWorld(earth);
  const floor = new RigidBody({ bodyType: "static" });
  const mesh = new Mesh(new BoxGeometry(10, 1, 10));
  const distant = new Mesh(new BoxGeometry(1, 1, 1));
  distant.position.x = 20;
  floor.add(mesh, distant);
  const body = box();
  body.position.y = 2;
  world.scene.add(floor, body);
  for (let i = 0; i < 90; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(1, 1);
  floor.remove(mesh);
  for (let i = 0; i < 90; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(0);
});

it("holds a prepared distance joint at its maximum", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = -2;
  world.scene.add(body);
  const joint = new DistanceJoint({
    body0: null,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits: [0, 2],
  });
  world.scene.add(joint);
  world.update(world.fixedDelta);
  world.update(world.fixedDelta);
  expect(joint.getState().distance).toBeCloseTo(2, 1);
});

it("keeps captured anchors when a joint is disabled and enabled", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = 3;
  world.scene.add(body);
  const joint = new FixedJoint({ body0: null, body1: body });
  world.scene.add(joint);
  joint.position.y = 3;
  world.update(world.fixedDelta);
  joint.setEnabled(false);
  for (let i = 0; i < 15; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(3);
  joint.setEnabled(true);
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(3, 2);
});

it("uses world matrices for teleport and rejects nonrigid transforms", async () => {
  const world = await createWorld(earth);
  const body = box();
  world.scene.add(body);
  const matrix = new Matrix4().makeTranslation(2, 3, 4);
  world.update(world.fixedDelta);
  body.teleport(matrix);
  expect(body.matrixWorld.elements).toEqual(matrix.elements);
  expect(body.position.toArray()).toEqual([2, 3, 4]);
  expect(() => body.teleport(new Matrix4().makeScale(2, 1, 1))).toThrow(
    "unit scale",
  );
  expect(body.matrixWorld.elements).toEqual(matrix.elements);
});

it("teleports a body together with the assembly jointed to it", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const root = box();
  const child = box();
  child.position.x = 2;
  world.scene.add(root, child);
  const joint = new RevoluteJoint({ body0: root, body1: child, axis: "X" });
  world.scene.add(joint);
  joint.position.x = 1;
  child.setVelocity({ angular: new Vector3(8, 0, 0) });
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  const turned = joint.getState().position;
  expect(Math.abs(turned)).toBeGreaterThan(Math.PI);
  child.teleport(new Matrix4().makeTranslation(2, 5, 0));
  expect(root.position.y).toBeCloseTo(5);
  expect(child.position.y).toBeCloseTo(5);
  // The hinge sits inside the moved assembly, so its turn count carries over.
  expect(joint.getState().position).toBeCloseTo(turned, 3);
  world.update(world.fixedDelta);
  expect(root.position.y).toBeCloseTo(5);
});
