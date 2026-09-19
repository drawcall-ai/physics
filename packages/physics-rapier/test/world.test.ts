import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh, Vector3 } from "three";
import { DistanceJoint, FixedJoint, RigidBody } from "@drawcall/physics";
import { createWorld, box, earth } from "./fixtures.js";

it("adds bodies after stepping without resetting existing velocities or poses", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const first = box();
  first.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  const previous = first.position.x;
  const second = box();
  second.position.y = 4;
  world.update(world.fixedDelta);
  expect(first.position.x).toBeGreaterThan(previous);
  expect(first.getVelocity().linear.x).toBeCloseTo(2);
  expect(second.position.y).toBeCloseTo(4);
});

it("adds joints after bodies are already simulating", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = 4;
  world.update(world.fixedDelta);
  const anchor = body.position.y;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = anchor;
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(anchor, 2);
  body.dispose();
  expect(joint.disposed).toBe(true);
  expect(() => joint.getFrame(0, new Matrix4())).toThrow("disposed");
  world.update(world.fixedDelta);
});

it("updates collider geometry without resetting the body", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const floor = new RigidBody({ type: "static" });
  const mesh = new Mesh(new BoxGeometry(1, 1, 1));
  floor.add(mesh);
  const body = box();
  body.position.x = 3;
  world.update(world.fixedDelta);
  mesh.geometry = new BoxGeometry(8, 1, 1);
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(Math.abs(body.position.y)).toBeGreaterThan(0.8);
});

it("allows one attached world and disposes pending joints with their body", async () => {
  const first = await createWorld(earth);
  const body = box();
  await expect(createWorld(earth)).rejects.toThrow("already built");
  const joint = new FixedJoint({ body0: null, body1: body });
  body.dispose();
  first.update(first.fixedDelta);
  expect(joint.disposed).toBe(true);
  first.dispose();
  const next = box();
  const second = await createWorld(earth);
  second.update(second.fixedDelta);
  expect(next.position.y).toBeLessThan(0);
});

it("materializes bodies created by before-step callbacks", async () => {
  const world = await createWorld(earth);
  let body: RigidBody | undefined;
  const stop = world.onBeforeStep(() => {
    body = box();
    body.position.y = 2;
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
  const floor = new RigidBody({ type: "static" });
  const mesh = new Mesh(new BoxGeometry(10, 1, 10));
  const distant = new Mesh(new BoxGeometry(1, 1, 1));
  distant.position.x = 20;
  floor.add(mesh, distant);
  const body = box();
  body.position.y = 2;
  for (let i = 0; i < 90; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(1, 1);
  floor.remove(mesh);
  for (let i = 0; i < 90; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(0);
});

it("copies caller-owned joint frames", async () => {
  const world = await createWorld(earth);
  const body = box();
  const frame = new Matrix4();
  new FixedJoint({ body0: null, body1: body, frame0: frame, frame1: frame });
  world.update(world.fixedDelta);
  frame.makeTranslation(0, 1, 0);
  world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(0);
});

it("copies immutable distance limits without changing a prepared joint", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = -2;
  const limits: [number, number] = [0, 2];
  const joint = new DistanceJoint({
    body0: null,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits,
  });
  world.update(world.fixedDelta);
  limits[1] = 8;
  expect(joint.limits).toEqual([0, 2]);
  world.update(world.fixedDelta);
  expect(joint.getState().distance).toBeCloseTo(2, 1);
});

it("keeps captured anchors when a joint is disabled and enabled", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = 3;
  const joint = new FixedJoint({ body0: null, body1: body });
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
  const joint = new FixedJoint({ body0: null, body1: first });
  world.update(0);
  const source = new FixedJoint({ body0: null, body1: second });
  expect(() => joint.copy(source)).toThrow();
  source.dispose();
  for (let i = 0; i < 30; i++) world.update(world.fixedDelta);
  expect(first.position.y).toBeCloseTo(4, 1);
  expect(second.position.y).toBeLessThan(3);
  first.dispose();
  expect(joint.disposed).toBe(true);
});

it("uses one update path for preparation, fractional time, catch-up and explicit advancement", async () => {
  const world = await createWorld({ fixedDelta: 0.125, maxSubsteps: 2 });
  const body = box();
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
