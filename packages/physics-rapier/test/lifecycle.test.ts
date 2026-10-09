import { expect, it } from "vitest";
import { createWorld } from "./fixtures.js";
import { buildWorld } from "../src/index.js";
import { BoxGeometry, Group, Matrix4, Quaternion, Vector3 } from "three";
import {
  BoxCollider,
  RigidBody,
  RevoluteJoint,
  SphericalJoint,
  DistanceJoint,
  PrismaticJoint,
  JointDrive,
  MeshCollider,
} from "@drawcall/physics";

const setup = () => createWorld({ fixedDelta: 1 / 60 });

it("commands a body once it joins the world at the next update or query", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 }).add(new BoxCollider());
  const input = new Vector3(1, 0, 0);
  body.setVelocity({ linear: input });
  input.x = 99;
  expect(body.getVelocity().linear.x).toBe(1);
  world.scene.add(body);
  expect(() => body.applyImpulse(new Vector3(2, 0, 0))).toThrow(
    "has not joined a world yet",
  );
  expect(
    world.raycast(new Vector3(-4, 0, 0), new Vector3(1, 0, 0), 8),
  ).toMatchObject({ kind: "body", body });
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(2);
  body.getVelocity().linear.x = 99;
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(2 * world.fixedDelta);
});

it("initializes on a short update without advancing time and prepares before observers", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 });
  world.scene.add(body);
  body.add(new BoxCollider());
  body.setVelocity({ linear: new Vector3(1, 0, 0) });
  world.update(world.fixedDelta / 2);
  body.applyImpulse(new Vector3(2, 0, 0));
  expect(body.position.x).toBe(0);
  const next = new RigidBody({ mass: 2 });
  next.add(new BoxCollider());
  next.position.x = 10;
  world.scene.add(next);
  world.onBeforeStep(() => next.applyImpulse(new Vector3(2, 0, 0)));
  world.update(world.fixedDelta / 2);
  expect(body.position.x).toBeCloseTo(2 * world.fixedDelta);
  expect(next.getVelocity().linear.x).toBeCloseTo(1);
});

it("shares world-pose writeback before and after initialization and freezes reset state", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 2 });
  const parent = new Group().add(body);
  world.scene.add(parent);
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
    world.scene.add(body);
    const joint = new RevoluteJoint({
      body0: null,
      body1: body,
      axis,
      frame0: new Matrix4(),
      frame1: new Matrix4().makeTranslation(0, 1, 0),
    });
    world.scene.add(joint);
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

it("rejects removed, foreign and invalid operations", async () => {
  const world = await setup();
  const body = new RigidBody({ bodyType: "kinematic" });
  world.scene.add(body);
  world.update(0);
  expect(() => body.setVelocity({ linear: new Vector3(NaN, 0, 0) })).toThrow(
    "finite",
  );
  expect(() => body.applyForce(new Vector3(Infinity, 0, 0))).toThrow("finite");
  body.setKinematicTarget(new Matrix4());
  body.sleep();
  body.wake();
  await expect(buildWorld({ scene: world.scene })).rejects.toThrow(
    "another world",
  );
  const other = await setup();
  const foreign = new RigidBody({ bodyType: "kinematic" });
  other.scene.add(foreign);
  other.update(0);
  expect(() => world.getVelocity(foreign)).toThrow(
    "Physics object is outside the world's scene",
  );
  body.removeFromParent();
  world.update(0);
  expect(() => body.wake()).toThrow(
    "Physics object has not joined a world yet",
  );
  world.update(0);
});

for (const kind of ["spherical", "distance"] as const)
  it(`keeps ${kind} state continuous across backend sync with authored frame rotations`, async () => {
    const world = await setup();
    const body = new RigidBody();
    body.add(new BoxCollider());
    body.position.y = 2;
    body.rotation.z = 0.3;
    world.scene.add(body);
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
    world.scene.add(joint);
    const initial = joint.getState();
    world.update(0);
    expect(joint.getState()).toEqual(initial);
  });

it("preserves world scale through static teleport and reset under a nonuniform parent", async () => {
  const world = await setup();
  const body = new RigidBody({ bodyType: "static" });
  body.add(new BoxCollider());
  const parent = new Group().add(body);
  world.scene.add(parent);
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

it("applies forces for one step and keeps replacing the drive effort held at each step", async () => {
  const world = await setup();
  const body = new RigidBody({
    colliders: false,
    mass: 2,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  world.scene.add(body);
  const joint = new PrismaticJoint({ body0: null, body1: body, axis: "X" });
  world.scene.add(joint);
  const drive = new JointDrive({});
  joint.setDrive(drive);
  expect(joint.getState()).toEqual({ position: 0, velocity: 0 });
  world.update(0);
  body.applyForce(new Vector3(2, 0, 0));
  body.applyForce(new Vector3(4, 0, 0));
  drive.setTarget({ effort: 100 });
  world.update(world.fixedDelta / 2);
  expect(world.time).toBe(0);
  const unsubscribe = world.onBeforeStep(() => drive.setTarget({ effort: 2 }));
  world.update(world.fixedDelta / 2);
  unsubscribe();
  expect(body.getVelocity().linear.x).toBeCloseTo(4 * world.fixedDelta);
  // Forces last one step; the drive's effort target persists.
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(5 * world.fixedDelta);
});

it("clears commands on reset and removal, and replaces kinematic targets", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  world.scene.add(body);
  const discarded = new RigidBody({ mass: 1 }).add(new BoxCollider());
  world.scene.add(discarded);
  const kinematic = new RigidBody({ bodyType: "kinematic", colliders: false });
  world.scene.add(kinematic);
  world.update(0);
  body.applyImpulse(new Vector3(5, 0, 0));
  body.applyForce(new Vector3(5, 0, 0));
  body.sleep();
  body.wake();
  discarded.applyForce(new Vector3(1, 0, 0));
  discarded.removeFromParent();
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

it("preserves sleep, wake and impulse ordering", async () => {
  const world = await setup();
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider());
  world.scene.add(body);
  world.update(0);
  body.applyImpulse(new Vector3(5, 0, 0));
  body.sleep();
  expect(body.getVelocity().linear.x).toBe(0);
  body.wake();
  body.applyImpulse(new Vector3(1, 0, 0));
  expect(body.getVelocity().linear.x).toBeCloseTo(1);
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(world.fixedDelta);
});

it("rebuilds a mesh collider only for geometry edits marked with needsUpdate", async () => {
  const world = await setup();
  const geometry = new BoxGeometry();
  const body = new RigidBody({ bodyType: "static" });
  body.add(
    new MeshCollider({ approximation: "convexHull" }).setGeometry(geometry),
  );
  world.scene.add(body);
  world.update(world.fixedDelta);
  const hits = () =>
    world.raycast(new Vector3(5, 5, 0), new Vector3(0, -1, 0), 10) !== null;
  expect(hits()).toBe(false);
  const position = geometry.getAttribute("position");
  for (let i = 0; i < position.count; i++)
    position.setX(i, position.getX(i) + 5);
  world.update(world.fixedDelta);
  expect(hits()).toBe(false);
  position.needsUpdate = true;
  world.update(world.fixedDelta);
  expect(hits()).toBe(true);
});

it("keeps the simulated velocity of a body that leaves its world", async () => {
  const world = await setup();
  const leaving = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const staying = new RigidBody({ mass: 1 }).add(new BoxCollider());
  staying.position.x = 5;
  world.scene.add(leaving, staying);
  world.update(world.fixedDelta);
  leaving.applyImpulse(new Vector3(3, 0, 0));
  staying.applyImpulse(new Vector3(0, 0, 4));
  const simulated = leaving.getVelocity().linear;
  expect(simulated.x).toBeCloseTo(3);

  leaving.removeFromParent();
  world.update(0);
  expect(leaving.world).toBeUndefined();
  expect(leaving.getVelocity().linear).toEqual(simulated);

  world.reset();
  expect(staying.getVelocity().linear.toArray()).toEqual([0, 0, 0]);
  staying.applyImpulse(new Vector3(0, 0, 4));
  world.dispose();
  expect(staying.getVelocity().linear.z).toBeCloseTo(4);
});
