import { afterEach, expect, it } from "vitest";
import { Group, Matrix4, Quaternion, Vector3 } from "three";
import {
  BoxCollider,
  RigidBody,
  RevoluteJoint,
  SphericalJoint,
  DistanceJoint,
} from "@drawcall/physics";
import { setupWorld, type RapierWorld } from "../src/index.js";

const worlds: RapierWorld[] = [];
async function setup() {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  worlds.push(world);
  return world;
}
afterEach(() => {
  for (const world of worlds) world.dispose();
  worlds.length = 0;
});

it("reads and writes velocity while staged, then initializes from the completed scaled hierarchy", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 }).setVelocity({
    angular: new Vector3(0, 1, 0),
  });
  world.unregister(body);
  expect(body.getVelocity().linear.toArray()).toEqual([0, 0, 0]);
  const input = new Vector3(1, 0, 0);
  body.setVelocity({ linear: input });
  input.x = 99;
  body.getVelocity().angular.y = 99;
  expect(body.getVelocity().linear.x).toBe(1);
  expect(body.getVelocity().angular.y).toBe(1);
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(0);
  expect(() => body.applyImpulse(new Vector3(2, 0, 0))).toThrow(
    "world.update(0)",
  );
  body.add(new BoxCollider());
  body.position.set(1, 3, 0);
  const parent = new Group().add(body);
  parent.scale.setScalar(2);
  world.register(body);
  world.update(0);
  expect(body.getWorldPosition(new Vector3()).toArray()).toEqual([2, 6, 0]);
  expect(body.getVelocity().linear.x).toBe(2);
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(3);
  world.update(world.fixedDelta);
  expect(body.getWorldPosition(new Vector3()).x).toBeCloseTo(
    2 + 3 * world.fixedDelta,
  );
  expect(body.scale.distanceTo(new Vector3(1, 1, 1))).toBeLessThan(1e-6);
});

it("initializes on a short update without advancing time and prepares before observers", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 });
  body.add(new BoxCollider());
  body.setVelocity({ linear: new Vector3(1, 0, 0) });
  world.update(world.fixedDelta / 2);
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.position.x).toBe(0);
  const next = new RigidBody({ mass: 2 });
  next.add(new BoxCollider());
  next.position.x = 10;
  world.onBeforeStep(() => next.applyImpulse(new Vector3(2, 0, 0)));
  world.update(world.fixedDelta / 2);
  expect(body.position.x).toBeCloseTo(2 * world.fixedDelta);
  expect(next.getVelocity().linear.x).toBeCloseTo(1);
});

it("shares world-pose writeback before and after initialization and freezes reset state", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 });
  const parent = new Group().add(body);
  parent.position.set(3, 4, 5);
  parent.rotation.y = 0.4;
  parent.scale.setScalar(2);
  body.scale.set(1, 2, 3);
  const pose = new Matrix4().compose(
    new Vector3(4, 8, 12),
    new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), 0.8),
    new Vector3(1, 1, 1),
  );
  body.teleport(pose);
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.add(new BoxCollider());
  world.update(0);
  body.teleport(new Matrix4().makeTranslation(20, 20, 20));
  body.setVelocity({ linear: new Vector3(9, 0, 0) });
  body.setVelocity({ linear: new Vector3(100, 0, 0) });
  world.reset();
  expect(
    body.getWorldPosition(new Vector3()).distanceTo(new Vector3(4, 8, 12)),
  ).toBeLessThan(1e-5);
  expect(body.getVelocity().linear.x).toBe(2);
  expect(body.scale.distanceTo(new Vector3(1, 2, 3))).toBeLessThan(1e-6);
});

for (const axis of ["X", "Y", "Z"] as const) {
  it(`reads authored joint state with scaled anchors and ${axis} axis before backend sync`, async () => {
    const world = await setup();
    const body = new RigidBody().setVelocity({ angular: new Vector3(2, 3, 4) });
    body.position.set(2, 3, 4);
    body.scale.setScalar(2);
    body.add(new BoxCollider());
    const joint = new RevoluteJoint({
      body0: null,
      body1: body,
      axis,
      frame0: new Matrix4(),
      frame1: new Matrix4().makeTranslation(0, 1, 0),
    });
    const initial = joint.getState();
    expect(initial.velocity).toBeCloseTo(
      axis === "X" ? 2 : axis === "Y" ? 3 : 4,
    );
    world.update(0);
    const ready = joint.getState();
    for (const key of ["position", "velocity"] as const)
      expect(ready[key]).toBeCloseTo(initial[key], 5);
    joint.setEnabled(false);
    world.update(world.fixedDelta);
    expect(Number.isFinite(joint.getState().position)).toBe(true);
  });
}

it("rejects disposed, foreign, invalid, and premature simulation operations", async () => {
  const world = await setup();
  const body = new RigidBody({}).setType("kinematic");
  expect(() => body.setVelocity({ linear: new Vector3(NaN, 0, 0) })).toThrow(
    "finite",
  );
  expect(() => body.applyForce(new Vector3(Infinity, 0, 0))).toThrow("finite");
  expect(() => body.setKinematicTarget(new Matrix4())).toThrow(
    "world.update(0)",
  );
  expect(() => body.sleep()).toThrow("world.update(0)");
  const other = await setup();
  expect(() => other.getVelocity(body)).toThrow("another world");
  body.dispose();
  expect(() => body.getVelocity()).toThrow("disposed");
  expect(() => body.teleport(new Matrix4())).toThrow("disposed");
});

for (const kind of ["spherical", "distance"] as const) {
  it(`keeps ${kind} state continuous when authored frame rotations are unused`, async () => {
    const world = await setup();
    const body = new RigidBody();
    body.add(new BoxCollider());
    body.position.y = 2;
    body.rotation.z = 0.3;
    const options = {
      body0: null,
      body1: body,
      frame0: new Matrix4().makeRotationX(0.8),
      frame1: new Matrix4().makeRotationY(0.5),
    };
    const joint =
      kind === "spherical"
        ? new SphericalJoint(options)
        : new DistanceJoint({ ...options, limits: [0, 3] });
    const initial = joint.getState();
    world.update(0);
    const ready = joint.getState();
    expect(ready).toEqual(initial);
  });
}

it("preserves world scale through static teleport and reset under a nonuniform parent", async () => {
  const world = await setup();
  const body = new RigidBody({}).setType("static");
  body.add(new BoxCollider());
  const parent = new Group().add(body);
  parent.scale.set(2, 3, 4);
  world.update(0);
  const scale = body.getWorldScale(new Vector3());
  body.teleport(new Matrix4().makeRotationZ(Math.PI / 2));
  expect(body.getWorldScale(new Vector3()).distanceTo(scale)).toBeLessThan(
    1e-6,
  );
  world.update(world.fixedDelta);
  expect(body.getWorldScale(new Vector3()).distanceTo(scale)).toBeLessThan(
    1e-6,
  );
  world.reset();
  expect(body.getWorldScale(new Vector3()).distanceTo(scale)).toBeLessThan(
    1e-6,
  );
  expect(body.quaternion.angleTo(new Quaternion())).toBeLessThan(1e-6);
  const pose = body.matrixWorld.clone();
  expect(() => body.teleport(new Matrix4().makeRotationZ(Math.PI / 4))).toThrow(
    "shear",
  );
  expect(body.matrixWorld.elements).toEqual(pose.elements);
  world.update(world.fixedDelta);
  expect(body.matrixWorld.elements).toEqual(pose.elements);
});
