import { expect, it } from "vitest";
import { Group, Matrix4, Mesh, BoxGeometry, Vector3 } from "three";
import {
  BoxCollider,
  JointDrive,
  DistanceJoint,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
  SphericalJoint,
} from "@drawcall/physics";
import { createWorld, box, earth, steps } from "./fixtures.js";

it("updates manual body matrices when synchronizing simulated poses", async () => {
  const world = await createWorld(earth);
  const scene = new Group(),
    body = box();
  body.position.y = 2;
  body.updateMatrix();
  body.matrixAutoUpdate = false;
  scene.add(body);
  world.update(world.fixedDelta);
  expect(body.position.y).toBeLessThan(2);
  expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(
    body.position.y,
    6,
  );
});

it("rejects invalid drive targets before stepping", async () => {
  await createWorld(earth);
  const scene = new Group(),
    body = box();
  const hinge = new RevoluteJoint({ body0: null, body1: body });
  scene.add(body, hinge);
  const drive = new JointDrive({});
  hinge.setDrive(drive);
  expect(() => drive.setTarget({ effort: NaN })).toThrow("finite");
  expect(() => drive.setTarget({ effort: Infinity })).toThrow("finite");
});

it("distributes explicit mass over compound colliders", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const scene = new Group(),
    body = new RigidBody({ mass: 20 });
  const a = new Mesh(new BoxGeometry(1, 1, 1));
  a.position.x = -1;
  const b = new Mesh(new BoxGeometry(2, 1, 1));
  b.position.x = 1;
  body.add(a, b);
  scene.add(body);
  world.update(world.fixedDelta);
  body.applyImpulse(new Vector3(10, 0, 0));
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(0.5, 5);
  body.applyImpulse(new Vector3(0, 1, 0), new Vector3(3, 0, 0));
  world.update(world.fixedDelta);
  expect(body.getVelocity().angular.z).toBeGreaterThan(0);
});

it("honors collision membership and filter masks", async () => {
  const world = await createWorld(earth);
  const scene = new Group();
  const floor = new RigidBody({ colliders: false, type: "static" });
  floor.add(
    new BoxCollider({ size: [10, 1, 10] }).setCollisionGroups({
      membership: 1,
      filter: 1,
    }),
  );
  const falling = new RigidBody({ colliders: false, mass: 1 });
  falling.add(
    new BoxCollider().setCollisionGroups({ membership: 2, filter: 2 }),
  );
  falling.position.y = 2;
  scene.add(floor, falling);
  steps(world, 60);
  expect(falling.position.y).toBeLessThan(-1);
});

it("keeps the opening of a compound frame empty", async () => {
  const world = await createWorld(earth);
  const scene = new Group(),
    frame = new RigidBody({ type: "static" }),
    falling = box();
  for (const x of [-2, 2]) {
    const post = new Mesh(new BoxGeometry(0.2, 4, 0.2));
    post.position.set(x, 2, 0);
    frame.add(post);
  }
  const lintel = new Mesh(new BoxGeometry(4.2, 0.2, 0.2));
  lintel.position.y = 4;
  frame.add(lintel);
  falling.position.y = 2;
  scene.add(frame, falling);
  steps(world, 60);
  expect(falling.position.y).toBeLessThan(0);
});

it("releases disposed bodies and rejects further operations", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.dispose();
  world.update(world.fixedDelta);
  expect(() => body.applyImpulse(new Vector3(1, 0, 0))).toThrow("disposed");
});

it("drops a body onto a floor, resets its pose, and rejects disposed access", async () => {
  const world = await createWorld(earth);
  const scene = new Group();
  const floor = box("static"),
    falling = box();
  floor.position.y = -0.5;
  falling.position.y = 3;
  scene.add(floor, falling);
  steps(world);
  expect(falling.position.y).toBeCloseTo(0.5, 1);
  world.reset();
  expect(falling.position.y).toBe(3);
  world.dispose();
  expect(() => falling.getVelocity()).toThrow("disposed");
});

it("drives a hinge under rotated parents while preserving its anchor", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const scene = new Group(),
    assembly = new Group();
  assembly.position.set(5, 2, 3);
  assembly.rotation.z = 0.4;
  scene.add(assembly);
  const frame = box("static"),
    door = box();
  door.position.x = 1;
  const hinge = new RevoluteJoint({
    body0: frame,
    body1: door,
    limits: [0, 1.5],
  });
  hinge.setDrive(
    new JointDrive({ stiffness: 50, damping: 10 }).setTarget({
      position: 1,
    }),
  );
  hinge.position.x = 0.5;
  assembly.add(frame, door, hinge);
  steps(world, 240);
  expect(hinge.getState().position).toBeCloseTo(1, 1);
  const frame0 = hinge
    .getFrame(0, new Matrix4())
    .premultiply(frame.matrixWorld);
  const frame1 = hinge.getFrame(1, new Matrix4()).premultiply(door.matrixWorld);
  expect(
    new Vector3()
      .setFromMatrixPosition(frame0)
      .distanceTo(new Vector3().setFromMatrixPosition(frame1)),
  ).toBeLessThan(0.01);
});

it("drives a slider to its target and applies live material edits", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const scene = new Group(),
    body = box();
  scene.add(body);
  const slider = new PrismaticJoint({
    body0: null,
    body1: body,
    axis: "X",
    limits: [0, 2],
  });
  slider.setDrive(
    new JointDrive({ stiffness: 50, damping: 10 }).setTarget({
      position: 1,
    }),
  );
  scene.add(slider);
  steps(world);
  expect(body.position.x).toBeCloseTo(1, 1);
  body.setMaterial({ restitution: 0.5 });
  world.update(world.fixedDelta);
  body.setMaterial({});
  world.update(world.fixedDelta);
  expect(new Vector3().setFromMatrixPosition(body.matrixWorld).x).toBeCloseTo(
    1,
    1,
  );
});

it("holds bodies with fixed, spherical, and distance constraints", async () => {
  for (const kind of ["fixed", "spherical", "distance"]) {
    const world = await createWorld(earth);
    const scene = new Group(),
      body = box();
    body.position.y = -2;
    scene.add(body);
    const joint =
      kind === "fixed"
        ? new FixedJoint({ body0: null, body1: body })
        : kind === "spherical"
          ? new SphericalJoint({ body0: null, body1: body })
          : new DistanceJoint({
              body0: null,
              body1: body,
              frame0: new Matrix4(),
              frame1: new Matrix4(),
              limits: [0, 2],
            });
    scene.add(joint);
    steps(world);
    expect(body.position.y).toBeCloseTo(-2, 1);
    world.dispose();
  }
});

it("applies forces for one step and accepts kinematic targets", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const scene = new Group(),
    body = box(),
    kinematic = box("kinematic");
  kinematic.position.x = 5;
  scene.add(body, kinematic);
  world.update(world.fixedDelta);
  body.applyForce(new Vector3(60, 0, 0));
  world.update(world.fixedDelta);
  const velocity = body.getVelocity().linear.x;
  world.update(world.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(velocity);
  kinematic.setKinematicTarget(new Matrix4().makeTranslation(6, 0, 0));
  world.update(world.fixedDelta);
  expect(kinematic.position.x).toBeCloseTo(6);
});
