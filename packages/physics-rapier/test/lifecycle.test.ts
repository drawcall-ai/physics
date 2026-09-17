import { expect, it } from "vitest";
import { createWorld } from "./fixtures.js";
import { Group, Matrix4, Quaternion, Vector3 } from "three";
import {
  BoxCollider,
  RigidBody,
  RevoluteJoint,
  SphericalJoint,
  DistanceJoint,
  PrismaticJoint,
  JointDrive,
} from "@drawcall/physics";

const setup = () => createWorld({ fixedDelta: 1 / 60 });

it("reads and queries construction state without capturing unfinished scale or replaying impulses", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 });
  const input = new Vector3(1, 0, 0);
  body.setVelocity({ linear: input });
  input.x = 99;
  expect(body.getVelocity().linear.x).toBe(1);
  body.getVelocity().linear.x = 99;
  body.add(new BoxCollider());
  body.applyImpulse(new Vector3(2, 0, 0));
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(3);
  expect(
    world.raycast(new Vector3(-4, 0, 0), new Vector3(1, 0, 0), 8)?.body,
  ).toBe(body);
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.applyImpulse(new Vector3(2, 0, 0));
  body.position.set(1, 3, 0);
  const parent = new Group().add(body);
  parent.scale.setScalar(2);
  expect(body.getVelocity().linear.x).toBeCloseTo(3);
  world.update(world.fixedDelta);
  expect(body.getWorldPosition(new Vector3()).x).toBeCloseTo(
    2 + 3 * world.fixedDelta,
  );
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(3);
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

for (const axis of ["X", "Y", "Z"] as const)
  it(`reads authored joint state with scaled anchors and ${axis} axis before backend sync`, async () => {
    const world = await setup();
    const body = new RigidBody().setVelocity({
      angular: new Vector3(2, 3, 4),
    });
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

it("rejects disposed, foreign and invalid operations while accepting staged commands", async () => {
  const world = await setup();
  const body = new RigidBody({ type: "kinematic" });
  expect(() => body.setVelocity({ linear: new Vector3(NaN, 0, 0) })).toThrow(
    "finite",
  );
  expect(() => body.applyForce(new Vector3(Infinity, 0, 0))).toThrow("finite");
  body.setKinematicTarget(new Matrix4());
  body.sleep();
  body.wake();
  const other = await setup();
  expect(() => other.getVelocity(body)).toThrow("another world");
  body.dispose();
  expect(() => body.getVelocity()).toThrow("disposed");
  expect(() => body.teleport(new Matrix4())).toThrow("disposed");
  world.update(0);
});

for (const kind of ["spherical", "distance"] as const)
  it(`keeps ${kind} state continuous across backend sync with authored frame rotations`, async () => {
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
    expect(joint.getState()).toEqual(initial);
  });

it("preserves world scale through static teleport and reset under a nonuniform parent", async () => {
  const world = await setup();
  const body = new RigidBody({ type: "static" });
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

it("applies staged forces once and keeps replacing the drive effort held at each step", async () => {
  const world = await setup();
  const body = new RigidBody({
    colliders: false,
    mass: 2,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  const joint = new PrismaticJoint({ body0: null, body1: body, axis: "X" });
  const drive = new JointDrive({});
  joint.setDrive(drive);
  body.applyForce(new Vector3(2, 0, 0));
  body.applyForce(new Vector3(4, 0, 0));
  drive.setTarget({ effort: 100 });
  expect(joint.getState()).toEqual({ position: 0, velocity: 0 });
  world.update(0);
  world.update(world.fixedDelta / 2);
  expect(world.time).toBe(0);
  const unsubscribe = world.onBeforeStep(() => drive.setTarget({ effort: 2 }));
  world.update(world.fixedDelta / 2);
  unsubscribe();
  expect(body.getVelocity().linear.x).toBeCloseTo(4 * world.fixedDelta);
  // Staged forces last one step; the drive's effort target persists.
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(5 * world.fixedDelta);
});

it("clears staged commands on reset and disposal, and replaces kinematic targets without replay", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  body.applyImpulse(new Vector3(5, 0, 0));
  body.applyForce(new Vector3(5, 0, 0));
  body.sleep();
  body.wake();
  const discarded = new RigidBody({ mass: 1 });
  discarded.applyForce(new Vector3(1, 0, 0));
  discarded.dispose();
  const kinematic = new RigidBody({ type: "kinematic", colliders: false });
  kinematic.setKinematicTarget(new Matrix4().makeTranslation(5, 0, 0));
  world.reset();
  expect(body.getVelocity().linear.x).toBe(0);
  kinematic.setKinematicTarget(new Matrix4().makeTranslation(2, 0, 0));
  kinematic.setKinematicTarget(new Matrix4().makeTranslation(3, 0, 0));
  world.update(world.fixedDelta);
  expect(body.position.x).toBe(0);
  expect(kinematic.position.x).toBeCloseTo(3);
  world.reset();
  world.update(world.fixedDelta);
  expect(kinematic.position.x).toBeCloseTo(0);
});

it("recomputes rotating slider reads after construction edits without capturing anchors", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 1 });
  const collider = new BoxCollider();
  collider.position.x = 1;
  body.add(collider);
  body.setVelocity({ angular: new Vector3(0, 0, 2) });
  const joint = new PrismaticJoint({ body0: null, body1: body, axis: "Y" });
  expect(joint.getState().velocity).toBeCloseTo(-2);
  collider.position.x = 2;
  body.scale.setScalar(2);
  expect(joint.getState().velocity).toBeCloseTo(-8);
  joint.position.y = 1;
  expect(joint.getState().position).toBeCloseTo(0);
  world.update(world.fixedDelta);
  expect(Number.isFinite(joint.getState().velocity)).toBe(true);
});

it("preserves staged sleep, wake and impulse ordering", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  body.applyImpulse(new Vector3(5, 0, 0));
  body.sleep();
  expect(body.getVelocity().linear.x).toBe(0);
  body.wake();
  body.applyImpulse(new Vector3(1, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(1);
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(world.fixedDelta);
});
