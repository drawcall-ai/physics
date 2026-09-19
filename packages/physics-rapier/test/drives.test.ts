import { expect, it } from "vitest";
import { Matrix4, Vector3 } from "three";
import {
  DistanceJoint,
  GenericJoint,
  JointDrive,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { createWorld, inertialBody } from "./fixtures.js";

it("keeps an untargeted drive passive, brakes at zero velocity, and removes actuation when detached", async () => {
  const world = await createWorld();
  const moving = inertialBody().setVelocity({
    linear: new Vector3(1, 0, 0),
  });
  const slider = new PrismaticJoint({
    body0: null,
    body1: moving,
    axis: "X",
  });
  const drive = new JointDrive({ damping: 10 });
  slider.setDrive(drive);
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(1, 5);
  drive.setTarget({ velocity: 0 });
  world.update(0.03);
  const braking = slider.getState().velocity;
  expect(braking).toBeLessThan(0.8);
  drive.setTarget(undefined);
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(braking, 5);
  drive.setTarget({ velocity: 0 });
  world.update(0.03);
  expect(slider.getState().velocity).toBeLessThan(braking);
  slider.setDrive(undefined);
  const released = slider.getState().velocity;
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(released, 5);
  expect(drive.joint).toBeUndefined();
});

it("drives hinge and slider position natively while reporting physical limits", async () => {
  const world = await createWorld();
  const hinge = new RevoluteJoint({
    body0: null,
    body1: inertialBody(),
    axis: "Z",
    limits: [-0.25, 0.25],
  });
  const slider = new PrismaticJoint({
    body0: null,
    body1: inertialBody(),
    axis: "X",
    limits: [-2, 2],
  });
  hinge.setDrive(
    new JointDrive({ stiffness: 100, damping: 10 }).setTarget({
      position: 1,
    }),
  );
  const drive = new JointDrive({ stiffness: 100, damping: 10 }).setTarget({
    position: 1,
  });
  slider.setDrive(drive);
  for (let i = 0; i < 200; i++) world.update(0.01);
  expect(hinge.getState().position).toBeCloseTo(0.25, 3);
  expect(hinge.getState().velocity).toBeCloseTo(0, 3);
  expect(slider.getState().position).toBeCloseTo(1, 2);
  drive.setTarget({ position: -1 });
  for (let i = 0; i < 200; i++) world.update(0.01);
  expect(slider.getState().position).toBeCloseTo(-1, 2);
});

it("limits native drive force and torque independently of timestep and model", async () => {
  const models: ("force" | "acceleration")[] = ["force", "acceleration"];
  for (const model of models)
    for (const Joint of [RevoluteJoint, PrismaticJoint])
      for (const dt of [0.01, 0.02]) {
        const world = await createWorld({ fixedDelta: dt });
        const joint = new Joint({
          body0: null,
          body1: inertialBody({ mass: 2, diagonalInertia: [2, 2, 2] }),
          axis: "Z",
        });
        joint.setDrive(
          new JointDrive({ model, damping: 1000, maxForce: 2 }).setTarget({
            velocity: 100,
          }),
        );
        world.update(dt);
        expect(joint.getState().velocity).toBeCloseTo(dt, 5);
        world.dispose();
      }
});

it("makes acceleration-based drives mass independent", async () => {
  const models: ("force" | "acceleration")[] = ["force", "acceleration"];
  for (const model of models) {
    const world = await createWorld();
    const light = new PrismaticJoint({
      body0: null,
      body1: inertialBody(),
      axis: "X",
    });
    const heavy = new PrismaticJoint({
      body0: null,
      body1: inertialBody({ mass: 10, diagonalInertia: [10, 10, 10] }),
      axis: "X",
    });
    for (const joint of [light, heavy])
      joint.setDrive(
        new JointDrive({ model, damping: 10 }).setTarget({ velocity: 1 }),
      );
    world.update(world.fixedDelta);
    if (model === "acceleration")
      expect(heavy.getState().velocity).toBeCloseTo(
        light.getState().velocity,
        5,
      );
    else
      expect(heavy.getState().velocity).toBeLessThan(
        light.getState().velocity / 5,
      );
    world.dispose();
  }
});

it("adds the effort term to a velocity drive and caps it by the same maximum force", async () => {
  const world = await createWorld();
  const pushed = new PrismaticJoint({
    body0: null,
    body1: inertialBody(),
    axis: "X",
  });
  const capped = new PrismaticJoint({
    body0: null,
    body1: inertialBody(),
    axis: "X",
  });
  pushed.setDrive(new JointDrive({}).setTarget({ effort: 10 }));
  capped.setDrive(new JointDrive({ maxForce: 2 }).setTarget({ effort: 10 }));
  world.update(0.01);
  expect(pushed.getState().velocity).toBeCloseTo(0.1, 5);
  expect(capped.getState().velocity).toBeCloseTo(0.02, 5);
  const combined = new PrismaticJoint({
    body0: null,
    body1: inertialBody().setVelocity({ linear: new Vector3(1, 0, 0) }),
    axis: "X",
  });
  combined.setDrive(
    new JointDrive({ damping: 100 }).setTarget({ velocity: 1, effort: 10 }),
  );
  world.update(0.01);
  // The damper resists the feed-forward term within the same solve, so the gain is partial.
  expect(combined.getState().velocity).toBeGreaterThan(1.04);
  expect(combined.getState().velocity).toBeLessThan(1.1);
});

for (const direction of [-1, 1])
  it(`tracks nearby continuous targets across wraps and holds after several turns (${direction})`, async () => {
    const world = await createWorld();
    const body = inertialBody().setVelocity({
      angular: new Vector3(0, 0, direction * 4),
    });
    const hinge = new RevoluteJoint({
      body0: null,
      body1: body,
      axis: "Z",
    });
    const drive = new JointDrive({
      stiffness: 100,
      damping: 20,
      maxForce: 20,
    });
    hinge.setDrive(drive);
    for (let i = 0; i < 400; i++) world.update(0.01);
    const held = hinge.getState().position;
    expect(direction * held).toBeGreaterThan(4 * Math.PI);
    body.setVelocity({ angular: new Vector3() });
    drive.setTarget({ position: held });
    for (let i = 0; i < 100; i++) world.update(0.01);
    expect(hinge.getState().position).toBeCloseTo(held, 3);
    for (let increment = 1; increment <= 8; increment++) {
      const goal = held + direction * increment;
      drive.setTarget({ position: goal });
      for (let i = 0; i < 150; i++) world.update(0.01);
      expect(hinge.getState().position).toBeCloseTo(goal, 2);
    }
    expect(hinge.getState().velocity).toBeCloseTo(0, 2);
  });

it("drives distance joints as force-limited springs that keep their rope limit", async () => {
  const world = await createWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
  });
  const hang = (limits: [number, number], maxForce?: number) => {
    const hand = new RigidBody({ type: "kinematic", colliders: false });
    const body = inertialBody({ mass: 10 });
    body.position.y = -0.2;
    const joint = new DistanceJoint({
      body0: hand,
      body1: body,
      frame0: new Matrix4(),
      frame1: new Matrix4(),
      limits,
    });
    joint.setDrive(
      new JointDrive({
        model: "acceleration",
        stiffness: 1000,
        damping: 63,
        maxForce,
      }).setTarget({ position: 0 }),
    );
    return { body, joint };
  };
  const held = hang([0, Infinity]);
  const dropped = hang([0, Infinity], 50);
  const roped = hang([0, 0.5], 50);
  for (let i = 0; i < 240; i++) world.update(1 / 120);
  expect(held.body.position.y).toBeCloseTo(-9.81 / 1000, 3);
  expect(dropped.body.position.y).toBeLessThan(-5);
  expect(roped.body.position.y).toBeCloseTo(-0.5, 3);
  expect(roped.joint.getState().distance).toBeCloseTo(0.5, 3);
  expect(roped.joint.getState().velocity).toBeCloseTo(0, 3);
  held.joint.drive?.setTarget({ position: 0.3 });
  for (let i = 0; i < 240; i++) world.update(1 / 120);
  expect(held.joint.getState().distance).toBeCloseTo(0.3 + 9.81 / 1000, 3);
});

it("locks, limits, and frees generic joint axes and drives each axis independently", async () => {
  const world = await createWorld();
  const welded = inertialBody().setVelocity({
    linear: new Vector3(1, 0, 0),
    angular: new Vector3(0, 2, 0),
  });
  welded.position.y = 1;
  new GenericJoint({ body0: null, body1: welded });
  const hinged = inertialBody().setVelocity({
    angular: new Vector3(0, 0, 4),
  });
  const hinge = new GenericJoint({
    body0: null,
    body1: hinged,
    dofs: { rotZ: [-0.25, 0.25] },
  });
  const sliding = inertialBody().setVelocity({
    linear: new Vector3(0, 0, 1),
  });
  const slider = new GenericJoint({
    body0: null,
    body1: sliding,
    dofs: { transZ: "free" },
  });
  slider.setDrive(
    "transZ",
    new JointDrive({ stiffness: 100, damping: 20 }).setTarget({
      position: 0.5,
    }),
  );
  for (let i = 0; i < 300; i++) world.update(0.01);
  expect(welded.position.y).toBeCloseTo(1, 4);
  expect(welded.getVelocity().linear.length()).toBeLessThan(1e-3);
  expect(hinge.getState("rotZ").position).toBeCloseTo(0.25, 2);
  expect(hinge.getState("rotX").position).toBeCloseTo(0, 3);
  expect(slider.getState("transZ").position).toBeCloseTo(0.5, 2);
  expect(sliding.position.z).toBeCloseTo(0.5, 2);
  slider.setDrive("transZ", undefined);
  sliding.setVelocity({ linear: new Vector3(0, 0, -1) });
  for (let i = 0; i < 5; i++) world.update(world.fixedDelta);
  expect(sliding.position.z).toBeCloseTo(0.45, 3);
});

it("holds a body at a moving hand through linear drives on a free generic joint", async () => {
  const world = await createWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
  });
  const hand = new RigidBody({ type: "kinematic", colliders: false });
  const body = inertialBody({ mass: 5 });
  const joint = new GenericJoint({
    body0: hand,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    dofs: {
      transX: "free",
      transY: "free",
      transZ: "free",
      rotX: "free",
      rotY: "free",
      rotZ: "free",
    },
  });
  for (const axis of ["transX", "transY", "transZ"] as const)
    joint.setDrive(
      axis,
      new JointDrive({
        model: "acceleration",
        stiffness: 1000,
        damping: 63,
        maxForce: 500,
      }).setTarget({ position: 0 }),
    );
  for (let i = 0; i < 240; i++) {
    hand.setKinematicTarget(new Matrix4().makeTranslation(i / 120, 0, 0));
    world.update(1 / 120);
  }
  expect(body.position.x).toBeCloseTo(2, 1);
  expect(body.position.y).toBeCloseTo(-9.81 / 1000, 2);
  expect(body.getVelocity().angular.length()).toBeLessThan(1e-6);
});

it("rejects a capped drive that combines gains with effort, which Rapier caps separately", async () => {
  const world = await createWorld();
  const joint = new PrismaticJoint({
    body0: null,
    body1: inertialBody(),
    axis: "X",
  });
  joint.setDrive(
    new JointDrive({ damping: 1, maxForce: 2 }).setTarget({
      velocity: 1,
      effort: 1,
    }),
  );
  expect(() => world.update(0)).toThrow("cannot combine gains with effort");
});
