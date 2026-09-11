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
  world.step();
  const previous = first.position.x;
  const second = box();
  second.position.y = 4;
  world.step();
  expect(first.position.x).toBeGreaterThan(previous);
  expect(world.body(first).getVelocity().linear.x).toBeCloseTo(2);
  expect(second.position.y).toBeCloseTo(4);
  world.dispose();
});

it("adds joints after bodies are already simulating", async () => {
  const world = await setupWorld();
  const body = box();
  body.position.y = 4;
  world.step();
  const anchor = body.position.y;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = anchor;
  for (let i = 0; i < 60; i++) world.step();
  expect(body.position.y).toBeCloseTo(anchor, 2);
  body.dispose();
  expect(() => world.joint(joint).getState()).toThrow("not active");
  world.step();
  world.dispose();
});

it("updates collider geometry without resetting the body", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const floor = new RigidBody({ type: "static" });
  const mesh = new Mesh(new BoxGeometry(1, 1, 1));
  floor.add(mesh);
  const body = box();
  body.position.x = 3;
  world.step();
  mesh.geometry = new BoxGeometry(8, 1, 1);
  for (let i = 0; i < 60; i++) world.step();
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
  first.step();
  expect(() => first.joint(joint).getState()).toThrow("not active");
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
  world.step();
  expect(body?.position.y).toBeLessThan(2);
  world.dispose();
});

it("updates live mass and type while retaining controls", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const body = box();
  const controls = world.body(body);
  body.options.mass = 2;
  world.step();
  controls.applyImpulse(new Vector3(2, 0, 0));
  expect(controls.getVelocity().linear.x).toBeCloseTo(1);
  body.options.type = "static";
  world.step();
  const x = body.position.x;
  world.step();
  expect(body.position.x).toBe(x);
  body.options.type = "dynamic";
  body.options.gravityScale = 0;
  body.options.linearDamping = 1;
  world.step();
  controls.setVelocity({ linear: new Vector3(1, 0, 0) });
  world.step();
  expect(controls.getVelocity().linear.x).toBeLessThan(1);
  body.options.canSleep = false;
  expect(() => world.step()).toThrow("canSleep cannot change");
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
  for (let i = 0; i < 90; i++) world.step();
  expect(body.position.y).toBeCloseTo(1, 1);
  floor.remove(mesh);
  for (let i = 0; i < 90; i++) world.step();
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
  world.step();
  joint.options.frame0 = new Matrix4().makeTranslation(0, 1, 0);
  expect(() => world.step()).toThrow("Joint frames cannot change");
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
  const controls = world.joint(joint);
  world.step();
  joint.options.limits = [1, 2];
  expect(() => world.step()).toThrow("zero minimum");
  expect(controls.getState().distance).toBeCloseTo(2, 1);
  joint.options.limits = [0, 2];
  world.step();
  expect(controls.getState().distance).toBeCloseTo(2, 1);
  world.dispose();
});

it("keeps captured anchors when a joint is disabled and enabled", async () => {
  const world = await setupWorld();
  const body = box();
  body.position.y = 3;
  const joint = new FixedJoint({ body0: null, body1: body });
  joint.position.y = 3;
  const controls = world.joint(joint);
  world.step();
  joint.options.enabled = false;
  for (let i = 0; i < 15; i++) world.step();
  expect(body.position.y).toBeLessThan(3);
  expect(() => controls.getState()).toThrow("not active");
  joint.options.enabled = true;
  for (let i = 0; i < 60; i++) world.step();
  expect(body.position.y).toBeCloseTo(3, 2);
  expect(controls.getState().distance).toBeLessThan(0.01);
  world.dispose();
});

it("uses world matrices for teleport and rejects nonrigid transforms", async () => {
  const world = await setupWorld();
  const body = box();
  const controls = world.body(body);
  const matrix = new Matrix4().makeTranslation(2, 3, 4);
  world.step();
  controls.teleport(matrix);
  const target = new Matrix4();
  expect(controls.getMatrix(target)).toBe(target);
  expect(target.elements).toEqual(matrix.elements);
  expect(body.position.toArray()).toEqual([2, 3, 4]);
  expect(() => controls.teleport(new Matrix4().makeScale(2, 1, 1))).toThrow(
    "unit scale",
  );
  expect(controls.getMatrix().elements).toEqual(matrix.elements);
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
    world.step();
    const source = new FixedJoint({ body0: null, body1: second });
    joint.copy(source);
    source.dispose();
    for (let i = 0; i < 30; i++) world.step();
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
