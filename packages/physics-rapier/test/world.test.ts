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

it("keeps creation options immutable while damping and gravity settings update live", async () => {
  const world = await createWorld(earth);
  const options = { mass: 2 };
  const body = new RigidBody(options);
  body.add(new Mesh(new BoxGeometry()));
  options.mass = 9;
  world.scene.add(body);
  world.update(0);
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(1);
  body.setGravityScale(0);
  body.setLinearDamping(1);
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeLessThan(1);
  expect(body.getVelocity().linear.y).toBe(0);
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

it("copies caller-owned joint frames", async () => {
  const world = await createWorld(earth);
  const body = box();
  world.scene.add(body);
  const frame = new Matrix4();
  world.scene.add(
    new FixedJoint({ body0: null, body1: body, frame0: frame, frame1: frame }),
  );
  world.update(world.fixedDelta);
  frame.makeTranslation(0, 1, 0);
  world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(0);
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

it("rejects copying joint identity and preserves the live constraint", async () => {
  const world = await createWorld(earth);
  const first = box();
  first.position.set(0, 4, 0);
  const second = box();
  second.position.set(3, 4, 0);
  world.scene.add(first, second);
  const joint = new FixedJoint({ body0: null, body1: first });
  world.scene.add(joint);
  world.update(0);
  const source = new FixedJoint({ body0: null, body1: second });
  world.scene.add(source);
  expect(() => joint.copy(source)).toThrow();
  source.removeFromParent();
  for (let i = 0; i < 30; i++) world.update(world.fixedDelta);
  expect(first.position.y).toBeCloseTo(4, 1);
  expect(second.position.y).toBeLessThan(3);
});

it("uses one update path for preparation, fractional time, catch-up and explicit advancement", async () => {
  const world = await createWorld({ fixedDelta: 0.125, maxSubsteps: 2 });
  const body = box();
  world.scene.add(body);
  body.setVelocity({ linear: new Vector3(1, 0, 0) });
  const steps: number[] = [];
  world.onAfterStep((delta) => steps.push(delta));
  world.update(0);
  body.applyImpulse(new Vector3(0, 0, 0));
  expect(steps).toEqual([]);
  world.update(world.fixedDelta / 2);
  expect(body.position.x).toBe(0);
  world.update(world.fixedDelta / 2);
  expect(body.position.x).toBeCloseTo(0.125);
  world.update(10);
  expect(steps).toEqual([0.125, 0.125, 0.125]);
  expect(body.position.x).toBeCloseTo(0.375);
  world.update(0);
  expect(steps).toHaveLength(3);
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(0.5);
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
