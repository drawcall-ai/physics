import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh, Vector3 } from "three";
import { DistanceJoint, FixedJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "../src/index.js";

function box() {
  const body = new RigidBody({ mass: 1 });
  body.add(new Mesh(new BoxGeometry(1, 1, 1)));
  return body;
}

it("adds bodies after stepping without resetting existing velocities or poses", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const first = box();
  first.options.linearVelocity = [2, 0, 0];
  world.update(world.fixedDelta);
  const previous = first.position.x;
  const second = box();
  second.position.y = 4;
  world.update(world.fixedDelta);
  expect(first.position.x).toBeGreaterThan(previous);
  expect(first.getVelocity().linear.x).toBeCloseTo(2);
  expect(second.position.y).toBeCloseTo(4);
  world.dispose();
});

it("adds joints after bodies are already simulating", async () => {
  const world = await setupWorld();
  const body = box();
  body.position.y = 4;
  world.update(world.fixedDelta);
  const anchor = body.position.y;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = anchor;
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(anchor, 2);
  body.dispose();
  expect(() => joint.getState()).toThrow("disposed");
  world.update(world.fixedDelta);
  world.dispose();
});

it("updates collider geometry without resetting the body", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const floor = new RigidBody({ type: "static" });
  const mesh = new Mesh(new BoxGeometry(1, 1, 1));
  floor.add(mesh);
  const body = box();
  body.position.x = 3;
  world.update(world.fixedDelta);
  mesh.geometry = new BoxGeometry(8, 1, 1);
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(Math.abs(body.position.y)).toBeGreaterThan(0.8);
  world.dispose();
});

it("captures worlds at construction and disposes pending joints with their body", async () => {
  const first = await setupWorld();
  const a = box();
  const second = await setupWorld();
  const b = box();
  expect(a.world).toBe(first);
  expect(b.world).toBe(second);
  expect(() => new FixedJoint({ body0: a, body1: b })).toThrow("world");
  const joint = new FixedJoint({ body0: null, body1: a });
  a.dispose();
  first.update(first.fixedDelta);
  expect(() => joint.getState()).toThrow("disposed");
  first.dispose();
  expect(box().world).toBe(second);
  second.dispose();
  expect(() => box()).toThrow("setupWorld");
});

it("materializes bodies created by before-step callbacks", async () => {
  const world = await setupWorld();
  let body: RigidBody | undefined;
  const stop = world.onBeforeStep(() => {
    body = box();
    body.position.y = 2;
    stop();
  });
  world.update(world.fixedDelta);
  expect(body?.position.y).toBeLessThan(2);
  world.dispose();
});

it("updates live mass and type without replacing the body", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const body = box();
  body.options.mass = 2;
  world.update(world.fixedDelta);
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(1);
  body.options.type = "static";
  world.update(world.fixedDelta);
  const x = body.position.x;
  world.update(world.fixedDelta);
  expect(body.position.x).toBe(x);
  body.options.type = "dynamic";
  body.options.gravityScale = 0;
  body.options.linearDamping = 1;
  world.update(world.fixedDelta);
  body.setVelocity({ linear: new Vector3(1, 0, 0) });
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeLessThan(1);
  body.options.canSleep = false;
  expect(() => world.update(world.fixedDelta)).toThrow(
    "canSleep cannot change",
  );
  world.dispose();
});

it("removes automatic colliders when geometry is removed", async () => {
  const world = await setupWorld();
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
  world.dispose();
});

it("rejects edits to captured joint frames", async () => {
  const world = await setupWorld();
  const body = box();
  const joint = new FixedJoint({
    body0: null,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  });
  world.update(world.fixedDelta);
  joint.options.frame0 = new Matrix4().makeTranslation(0, 1, 0);
  expect(() => world.update(world.fixedDelta)).toThrow(
    "Joint frames cannot change",
  );
  world.dispose();
});

it("preserves a live joint when invalid replacement settings fail", async () => {
  const world = await setupWorld();
  const body = box();
  body.position.y = -2;
  const joint = new DistanceJoint({
    body0: null,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits: [0, 2],
  });
  world.update(world.fixedDelta);
  joint.options.limits = [1, 2];
  expect(() => world.update(world.fixedDelta)).toThrow("zero minimum");
  expect(joint.getState().distance).toBeCloseTo(2, 1);
  joint.options.limits = [0, 2];
  world.update(world.fixedDelta);
  expect(joint.getState().distance).toBeCloseTo(2, 1);
  world.dispose();
});

it("keeps captured anchors when a joint is disabled and enabled", async () => {
  const world = await setupWorld();
  const body = box();
  body.position.y = 3;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = 3;
  world.update(world.fixedDelta);
  joint.options.enabled = false;
  for (let i = 0; i < 15; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(3);
  expect(joint.getState().distance).toBeGreaterThanOrEqual(0);
  joint.options.enabled = true;
  for (let i = 0; i < 60; i++) world.update(world.fixedDelta);
  expect(body.position.y).toBeCloseTo(3, 2);
  expect(joint.getState().distance).toBeLessThan(0.01);
  world.dispose();
});

it("uses world matrices for teleport and rejects nonrigid transforms", async () => {
  const world = await setupWorld();
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
  world.dispose();
});

it("reconnects a copied live joint without unregistering the authoring object", async () => {
  const world = await setupWorld();
  try {
    const first = box();
    first.position.set(0, 4, 0);
    const second = box();
    second.position.set(3, 4, 0);
    const joint = new FixedJoint({ body0: null, body1: first });
    world.update(world.fixedDelta);
    const source = new FixedJoint({ body0: null, body1: second });
    joint.copy(source);
    source.dispose();
    for (let i = 0; i < 30; i++) world.update(world.fixedDelta);
    expect(first.position.y).toBeLessThan(3);
    expect(second.position.y).toBeCloseTo(4, 1);
    second.dispose();
    expect(joint.disposed).toBe(true);
  } finally {
    world.dispose();
  }
});

for (const solverIterations of [0, -1, 1.5, NaN, Infinity]) {
  it(`rejects invalid solver iteration count ${solverIterations}`, async () => {
    await expect(setupWorld({ solverIterations })).rejects.toThrow(
      "solverIterations must be a positive integer",
    );
  });
}

it("uses one update path for preparation, fractional time, catch-up and explicit advancement", async () => {
  const world = await setupWorld({
    gravity: [0, 0, 0],
    fixedDelta: 0.125,
    maxSubsteps: 2,
  });
  try {
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
  } finally {
    world.dispose();
  }
});
