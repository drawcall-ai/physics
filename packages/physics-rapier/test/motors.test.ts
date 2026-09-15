import { expect, it } from "vitest";
import { Vector3 } from "three";
import {
  JointMotor,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { setupWorld } from "../src/index.js";

function body(mass = 1) {
  return new RigidBody({
    colliders: false,
    mass,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [mass, mass, mass],
  });
}

it("keeps an untargeted motor passive, brakes at zero velocity, and removes actuation when disabled or disposed", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  const moving = body().setVelocity({ linear: new Vector3(1, 0, 0) });
  const slider = new PrismaticJoint({ body0: null, body1: moving, axis: "X" });
  const motor = new JointMotor({ joint: slider, damping: 10 });
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(1, 5);
  motor.setTarget({ velocity: 0 });
  world.update(0.03);
  const braking = slider.getState().velocity;
  expect(braking).toBeLessThan(0.8);
  motor.setEnabled(false);
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(braking, 5);
  motor.setEnabled(true);
  world.update(0.03);
  expect(slider.getState().velocity).toBeLessThan(braking);
  motor.dispose();
  const released = slider.getState().velocity;
  world.update(0.03);
  expect(slider.getState().velocity).toBeCloseTo(released, 5);
  expect(slider.disposed).toBe(false);
  world.dispose();
});

it("drives hinge and slider position natively while reporting physical limits", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  const hinge = new RevoluteJoint({
    body0: null,
    body1: body(),
    axis: "Z",
    limits: [-0.25, 0.25],
  });
  const slider = new PrismaticJoint({
    body0: null,
    body1: body(),
    axis: "X",
    limits: [-2, 2],
  });
  new JointMotor({ joint: hinge, stiffness: 100, damping: 10 }).setTarget({
    position: 1,
  });
  const motor = new JointMotor({
    joint: slider,
    stiffness: 100,
    damping: 10,
  }).setTarget({ position: 1 });
  for (let i = 0; i < 200; i++) world.update(0.01);
  expect(hinge.getState().position).toBeCloseTo(0.25, 3);
  expect(hinge.getState().velocity).toBeCloseTo(0, 3);
  expect(slider.getState().position).toBeCloseTo(1, 2);
  motor.setTarget({ position: -1 });
  for (let i = 0; i < 200; i++) world.update(0.01);
  expect(slider.getState().position).toBeCloseTo(-1, 2);
  world.dispose();
});

it("limits native motor force and torque independently of timestep and model", async () => {
  const models: ("force" | "acceleration")[] = ["force", "acceleration"];
  for (const model of models)
    for (const rotary of [false, true])
      for (const dt of [0.01, 0.02]) {
        const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: dt });
        const joint = rotary
          ? new RevoluteJoint({ body0: null, body1: body(2), axis: "Z" })
          : new PrismaticJoint({ body0: null, body1: body(2), axis: "X" });
        new JointMotor({ joint, model, damping: 1000, maxForce: 2 }).setTarget({
          velocity: 100,
        });
        world.update(dt);
        expect(joint.getState().velocity).toBeCloseTo(dt, 5);
        world.dispose();
      }
});

it("preserves Rapier acceleration-based motor behavior across body masses", async () => {
  const models: ("force" | "acceleration")[] = ["force", "acceleration"];
  for (const model of models) {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
    const light = new PrismaticJoint({
      body0: null,
      body1: body(1),
      axis: "X",
    });
    const heavy = new PrismaticJoint({
      body0: null,
      body1: body(10),
      axis: "X",
    });
    new JointMotor({ joint: light, model, damping: 10 }).setTarget({
      velocity: 1,
    });
    new JointMotor({ joint: heavy, model, damping: 10 }).setTarget({
      velocity: 1,
    });
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

it("rejects motor/effort conflicts before applying any forces and permits explicit cancellation", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  const first = new PrismaticJoint({ body0: null, body1: body(), axis: "X" });
  const second = new PrismaticJoint({ body0: null, body1: body(), axis: "X" });
  const motor = new JointMotor({ joint: second, damping: 10 });
  world.update(0);
  first.setEffort(10);
  second.setEffort(20);
  motor.setTarget({ velocity: 1 });
  expect(() => second.setEffort(2)).toThrow("Disable");
  expect(() => world.update(0.01)).toThrow("Disable");
  expect(world.time).toBe(0);
  second.setEffort(0);
  motor.setEnabled(false);
  world.update(0);
  expect(first.getState().velocity).toBeCloseTo(0.1, 5);
  expect(second.getState().velocity).toBeCloseTo(0, 5);
  world.dispose();
});
