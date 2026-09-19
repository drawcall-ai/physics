import { afterEach, expect, test } from "vitest";
import {
  BoxCollider,
  SphereCollider,
  RigidBody,
  Trigger,
  RevoluteJoint,
  PrismaticJoint,
  FixedJoint,
  SphericalJoint,
  GenericJoint,
  DistanceJoint,
  JointDrive,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import {
  buildWorld,
  type MujocoWorld,
  type MujocoOptions,
} from "../src/index.js";
const worlds: MujocoWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function world(options: MujocoOptions = {}) {
  const value = await buildWorld({
    gravity: [0, 0, 0],
    fixedDelta: 0.01,
    ...options,
  });
  worlds.push(value);
  return value;
}
function body(type: "dynamic" | "static" | "kinematic" = "dynamic") {
  const result = new RigidBody({ type, mass: 2 });
  result.add(new BoxCollider());
  return result;
}
function steps(value: MujocoWorld, count: number) {
  for (let i = 0; i < count; i++) value.update(value.fixedDelta);
}

test("loads the real WASM in Node, falls, contacts, resets, and disposes", async () => {
  const value = await world({ gravity: [0, -10, 0] });
  const floor = body("static");
  floor.scale.set(10, 1, 10);
  floor.position.y = -0.5;
  const box = body();
  box.position.y = 2;
  let contacts = 0;
  box.addEventListener("contactbegin", () => contacts++);
  steps(value, 200);
  expect(box.position.y).toBeCloseTo(0.5, 3);
  expect(contacts).toBeGreaterThan(0);
  value.reset();
  expect(box.position.y).toBe(2);
  expect(value.time).toBe(0);
  value.dispose();
  expect(box.disposed).toBe(true);
  expect(() => value.update(0)).toThrow("disposed");
});

test("force lasts one step, impulses act at the COM or at a world point", async () => {
  const value = await world();
  const box = body();
  box.applyForce(new Vector3(20, 0, 0));
  steps(value, 2);
  expect(box.getVelocity().linear.x).toBeCloseTo(0.1);
  box.applyImpulse(new Vector3(2, 0, 0));
  expect(box.getVelocity().linear.x).toBeCloseTo(1.1);
  box.applyImpulse(new Vector3(0, 2, 0), new Vector3(1, 0, 0));
  expect(box.getVelocity().angular.z).toBeGreaterThan(1);
});

test("queries exact primitives and collision masks, including triggers", async () => {
  const value = await world();
  const box = body();
  box.position.x = 3;
  const trigger = new Trigger();
  trigger.position.x = 1;
  trigger.add(new SphereCollider({ radius: 0.4 }));
  const ray = () => value.raycast(new Vector3(), new Vector3(1, 0, 0), 10);
  expect(ray()?.distance).toBeCloseTo(2.5);
  expect(
    value.raycast(new Vector3(), new Vector3(2, 0, 0), 10, {
      includeTriggers: true,
    })?.kind,
  ).toBe("trigger");
  expect(
    value.raycast(new Vector3(), new Vector3(1, 0, 0), 10, {
      excludeBodies: [box],
    }),
  ).toBeNull();
  box.setCollisionGroups({ membership: 1, filter: 2 });
  expect(
    value.raycast(new Vector3(), new Vector3(1, 0, 0), 10, {
      collisionGroups: { membership: 4, filter: 1 },
    }),
  ).toBeNull();
  box.teleport(new Matrix4().makeTranslation(5, 0, 0));
  expect(ray()?.distance).toBeCloseTo(4.5);
});

test("triggers emit aggregate enter/exit events without contact forces", async () => {
  const value = await world();
  const box = body();
  const trigger = new Trigger();
  trigger.add(new BoxCollider({ size: [2, 2, 2] }));
  const events: string[] = [];
  trigger.addEventListener("enter", () => events.push("enter"));
  trigger.addEventListener("exit", () => events.push("exit"));
  value.update(0.01);
  expect(trigger.overlaps(box)).toBe(true);
  box.teleport(new Matrix4().makeTranslation(4, 0, 0));
  value.update(0.01);
  expect(events).toEqual(["enter", "exit"]);
  expect(box.getVelocity().linear.length()).toBe(0);
});

test("preserves live state when adding and removing objects or editing material", async () => {
  const value = await world();
  const first = body().setVelocity({ linear: new Vector3(2, 0, 0) });
  value.update(0.01);
  const second = body();
  second.position.y = 5;
  first.setMaterial({ staticFriction: 0.8, dynamicFriction: 0.8 });
  value.update(0.01);
  expect(first.position.x).toBeCloseTo(0.04);
  expect(first.getVelocity().linear.x).toBeCloseTo(2);
  second.dispose();
  value.update(0.01);
  expect(first.position.x).toBeCloseTo(0.06);
});

for (const Type of [RevoluteJoint, PrismaticJoint])
  test(`${Type.name} drive targets, effort limits, and disable/re-enable`, async () => {
    const value = await world();
    const box = body();
    const joint = new Type({
      body0: null,
      body1: box,
      axis: "X",
      frame0: new Matrix4(),
      frame1: new Matrix4(),
      limits: [-1, 1],
    });
    const drive = new JointDrive({
      stiffness: 100,
      damping: 20,
      maxForce: 50,
    }).setTarget({ position: 0.5 });
    joint.setDrive(drive);
    steps(value, 300);
    expect(joint.getState().position).toBeCloseTo(0.5, 2);
    drive.setTarget({ position: -0.5 });
    steps(value, 300);
    expect(joint.getState().position).toBeCloseTo(-0.5, 2);
    joint.setEnabled(false);
    value.update(0.01);
    joint.setEnabled(true);
    value.update(0.01);
    expect(Number.isFinite(box.position.x)).toBe(true);
  });

test("fixed and spherical joints keep anchors together", async () => {
  const value = await world({ gravity: [0, -10, 0] });
  const fixed = body();
  fixed.position.x = -2;
  new FixedJoint({
    body0: null,
    body1: fixed,
    frame0: new Matrix4().makeTranslation(-2, 0, 0),
    frame1: new Matrix4(),
  });
  const ball = body();
  ball.position.set(2, -1, 0);
  const joint = new SphericalJoint({
    body0: null,
    body1: ball,
    frame0: new Matrix4().makeTranslation(2, 0, 0),
    frame1: new Matrix4().makeTranslation(0, 1, 0),
  });
  steps(value, 100);
  expect(fixed.position.y).toBeCloseTo(0);
  expect(value.readJoint(joint).translation.length()).toBeLessThan(1e-6);
});

test("distance tendon holds a rope length", async () => {
  const value = await world({ gravity: [0, -10, 0] });
  const box = body();
  box.position.y = -2;
  const joint = new DistanceJoint({
    body0: null,
    body1: box,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits: [2, 2],
  });
  steps(value, 100);
  expect(joint.getState().distance).toBeCloseTo(2, 2);
});

test("generic joint exposes a driven linear degree of freedom", async () => {
  const value = await world();
  const box = body();
  const joint = new GenericJoint({
    body0: null,
    body1: box,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    dofs: { transY: [-1, 1] },
  });
  joint.setDrive(
    "transY",
    new JointDrive({ stiffness: 100, damping: 20 }).setTarget({
      position: 0.5,
    }),
  );
  steps(value, 200);
  expect(joint.getState("transY").position).toBeCloseTo(0.5, 2);
});

test("validates options, ownership, scales, and unsupported closed joint chains", async () => {
  await expect(buildWorld({ fixedDelta: 0 })).rejects.toThrow("fixedDelta");
  const a = await world();
  const box = body();
  await expect(world()).rejects.toThrow("already built");
  a.update(0);
  box.scale.x = 2;
  expect(() => a.update(0)).toThrow("scale cannot change");
  box.scale.x = 1;
  new FixedJoint({ body0: null, body1: box });
  new FixedJoint({ body0: null, body1: box });
  expect(() => a.update(0)).toThrow("two parent joints");
});

test("step callbacks have committed times and can safely dispose the world", async () => {
  const value = await world({ maxSubsteps: 2 });
  body();
  const times: number[] = [];
  value.onAfterStep(() => times.push(value.time));
  value.update(1);
  expect(times).toEqual([0.01, 0.02]);
  value.onBeforeStep(() => value.dispose());
  value.update(1);
  expect(value.disposed).toBe(true);
});

test("preserves an initial hinge velocity and continuous turns across recompilation", async () => {
  const value = await world();
  const box = body().setVelocity({ angular: new Vector3(0, 0, 8) });
  const joint = new RevoluteJoint({
    body0: null,
    body1: box,
    axis: "Z",
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  });
  steps(value, 100);
  expect(joint.getState().position).toBeCloseTo(8, 3);
  box.setMaterial({ staticFriction: 0.7, dynamicFriction: 0.7 });
  steps(value, 100);
  expect(joint.getState().position).toBeCloseTo(16, 3);
  value.reset();
  steps(value, 100);
  expect(joint.getState().position).toBeCloseTo(8, 3);
});

test("moves attached triggers in the completed step and refreshes static raycasts", async () => {
  const value = await world();
  const moving = body().setVelocity({ linear: new Vector3(100, 0, 0) });
  const trigger = new Trigger();
  trigger.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
  moving.add(trigger);
  const other = body("static");
  other.position.x = 1;
  value.update(0.01);
  expect(trigger.overlaps(other)).toBe(true);
  other.position.x = 5;
  expect(
    value.raycast(new Vector3(2, 0, 0), new Vector3(1, 0, 0), 10)?.distance,
  ).toBeCloseTo(2.5);
});

test("generic angular drives use frame coordinates across scene rebuilds", async () => {
  const value = await world();
  const box = body();
  box.rotation.set(0.3, 0.2, -0.1);
  const joint = new GenericJoint({
    body0: null,
    body1: box,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    dofs: { rotX: "free", rotY: "free", rotZ: "free" },
  });
  for (const axis of ["rotX", "rotY", "rotZ"] as const)
    joint.setDrive(
      axis,
      new JointDrive({ stiffness: 10, damping: 4 }).setTarget({
        position: 0.4,
      }),
    );
  steps(value, 200);
  for (const axis of ["rotX", "rotY", "rotZ"] as const)
    expect(joint.getState(axis).position).toBeCloseTo(0.4, 2);
  box.setMaterial({ staticFriction: 0.7, dynamicFriction: 0.7 });
  steps(value, 100);
  for (const axis of ["rotX", "rotY", "rotZ"] as const)
    expect(joint.getState(axis).position).toBeCloseTo(0.4, 2);
});

test("raycasts leave initial scale editable and explicit mass supports zero density", async () => {
  const value = await world();
  const box = body();
  box.setMaterial({ density: 0 });
  expect(
    value.raycast(new Vector3(-3, 0, 0), new Vector3(1, 0, 0), 10)?.distance,
  ).toBeCloseTo(2.5);
  box.scale.setScalar(2);
  box.applyImpulse(new Vector3(2, 0, 0));
  value.update(0.01);
  expect(box.getVelocity().linear.x).toBeCloseTo(1);
});

test("scans a scene of many bodies without recompiling the model per ray", async () => {
  const value = await world({ gravity: [0, -9.81, 0] });
  for (let i = 0; i < 40; i++) {
    const shelf = body("static");
    shelf.scale.set(1, 2, 4);
    shelf.position.set((i % 8) * 3 - 12, 1, Math.floor(i / 8) * 4 - 8);
  }
  const moving = body();
  moving.position.set(0, 0.5, 0);
  steps(value, 1);
  const origin = new Vector3(0, 0.5, 0);
  const start = performance.now();
  for (let i = 0; i < 361; i++) {
    const angle = (i / 360) * Math.PI * 2;
    value.raycast(origin, new Vector3(Math.cos(angle), 0, Math.sin(angle)), 12);
  }
  // Compiling a fresh model per ray took tens of seconds for a single lidar scan.
  expect(performance.now() - start).toBeLessThan(1500);
});

test("raycasts see bodies added and moved between steps", async () => {
  const value = await world();
  const ray = () =>
    value.raycast(new Vector3(), new Vector3(1, 0, 0), 20)?.distance;
  const near = body("static");
  near.position.x = 3;
  steps(value, 1);
  expect(ray()).toBeCloseTo(2.5);
  near.position.x = 8;
  expect(ray()).toBeCloseTo(7.5);
  const added = body("static");
  added.position.x = 5;
  expect(ray()).toBeCloseTo(4.5);
});

for (const noSlipIterations of [-1, 1.5, NaN, Infinity]) {
  test(`rejects the no-slip iteration count ${noSlipIterations}`, async () => {
    await expect(world({ noSlipIterations })).rejects.toThrow(
      "noSlipIterations must be a nonnegative integer",
    );
  });
}
