import { expect, it } from "vitest";
import { Group, Matrix4, Vector3 } from "three";
import { JointDrive, PrismaticJoint, RevoluteJoint } from "@drawcall/physics";
import { createWorld, inertialBody } from "./fixtures.js";

it("prepares before observers and applies the effort held at step time for one substep", async () => {
  const world = await createWorld();
  const body = inertialBody();
  const joint = new PrismaticJoint({ body0: null, body1: body, axis: "X" });
  const other = new PrismaticJoint({ body0: null, body1: body, axis: "X" });
  const effort = new JointDrive({}).setTarget({ effort: 5 });
  joint.setDrive(effort);
  const idle = new JointDrive({}).setTarget({ effort: 20 });
  other.setDrive(idle);
  const stop = world.onBeforeStep(() => {
    body.applyForce(new Vector3(4, 0, 0));
    effort.setTarget({ effort: 2 }).setTarget({ effort: 3 });
    idle.setTarget(undefined);
    stop();
  });
  world.update(0.004);
  expect(world.time).toBe(0);
  world.update(0.006);
  expect(joint.getState().velocity).toBeCloseTo(0.07, 5);
  effort.setTarget(undefined);
  world.update(0.02);
  expect(joint.getState().velocity).toBeCloseTo(0.07, 5);
  effort.setTarget({ effort: 9 });
  joint.setEnabled(false);
  world.update(0.01);
  expect(joint.getState().velocity).toBeCloseTo(0.07, 5);
  joint.setEnabled(true);
  world.reset();
  effort.setTarget(undefined);
  world.update(0.01);
  expect(joint.getState().velocity).toBe(0);
});

for (const Joint of [RevoluteJoint, PrismaticJoint])
  it(`applies joint effort and reaction through transformed frames: ${Joint.name}`, async () => {
    const world = await createWorld();
    const parent = new Group();
    parent.position.set(2, 3, 4);
    parent.rotation.z = 0.7;
    parent.scale.setScalar(2);
    const first = inertialBody(),
      second = inertialBody();
    parent.add(first, second);
    const frame = new Matrix4().makeRotationY(Math.PI / 2).setPosition(0, 1, 0);
    const joint = new Joint({
      body0: first,
      body1: second,
      axis: "Z",
      frame0: frame,
      frame1: frame,
    });
    world.update(0);
    joint.setDrive(new JointDrive({}).setTarget({ effort: 2 }));
    world.update(0.01);
    const rotary = joint instanceof RevoluteJoint;
    const a = rotary ? first.getVelocity().angular : first.getVelocity().linear;
    const b = rotary
      ? second.getVelocity().angular
      : second.getVelocity().linear;
    expect(a.clone().add(b).length()).toBeLessThan(1e-6);
    expect(
      b.dot(new Vector3(1, 0, 0).applyQuaternion(parent.quaternion)),
    ).toBeGreaterThan(0);
    expect(joint.getState().velocity).toBeGreaterThan(0);
  });

it("measures offset COM and moving reference-axis velocity", async () => {
  const world = await createWorld({ fixedDelta: 0.0001 });
  const first = inertialBody().setVelocity({ angular: new Vector3(0, 0, 2) });
  const second = inertialBody({ centerOfMass: [1, 0, 0] }).setVelocity({
    angular: new Vector3(0, 0, 3),
  });
  second.position.set(2, 1, 0);
  const slider = new PrismaticJoint({
    body0: first,
    body1: second,
    axis: "X",
    frame0: new Matrix4().makeTranslation(0, 1, 0),
    frame1: new Matrix4().makeTranslation(0, 2, 0),
  }).setEnabled(false);
  world.update(0);
  const initial = slider.getState();
  expect(initial.velocity).toBeCloseTo(0, 6);
  world.update(world.fixedDelta);
  expect(
    (slider.getState().position - initial.position) / world.fixedDelta,
  ).toBeCloseTo(initial.velocity, 2);
  const offset = new PrismaticJoint({
    body0: null,
    body1: second,
    axis: "Y",
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  }).setEnabled(false);
  expect(offset.getState().velocity).toBeCloseTo(-3, 4);
});

for (const speed of [8, -8])
  it(`tracks turns without getters at ${speed} rad/s and rebases teleport/reset`, async () => {
    const world = await createWorld();
    const body = inertialBody().setVelocity({
      angular: new Vector3(0, 0, speed),
    });
    const hinge = new RevoluteJoint({
      body0: null,
      body1: body,
      axis: "Z",
    });
    for (let i = 0; i < 200; i++) world.update(0.01);
    expect(hinge.getState().position).toBeCloseTo(speed * 2, 1);
    body.teleport(new Matrix4().makeRotationZ(0.25));
    expect(hinge.getState().position).toBeCloseTo(0.25, 5);
    world.reset();
    expect(hinge.getState().position).toBeCloseTo(0, 5);
    expect(world.time).toBe(0);
  });

it("applies runtime damping/gravity settings while preserving the initialized velocity baseline", async () => {
  const world = await createWorld({ gravity: [0, -10, 0] });
  const body = inertialBody()
    .setVelocity({ linear: new Vector3(2, 0, 0) })
    .setGravityScale(0);
  world.update(0.01);
  expect(body.getVelocity().linear.toArray()).toEqual([2, 0, 0]);
  body
    .setVelocity({ linear: new Vector3(4, 0, 0) })
    .setLinearDamping(10)
    .setGravityScale(1);
  world.update(0.01);
  expect(body.getVelocity().linear.x).toBeLessThan(4);
  expect(body.getVelocity().linear.y).toBeLessThan(0);
  world.reset();
  expect(body.getVelocity().linear.toArray()).toEqual([2, 0, 0]);
});

it("commits time before observers and discards catch-up excess without replay", async () => {
  const world = await createWorld({ maxSubsteps: 2 });
  const observed: number[] = [];
  world.onBeforeStep(() => observed.push(world.time));
  const stop = world.onAfterStep(() => {
    throw new Error("observer");
  });
  expect(() => world.update(0.01)).toThrow("observer");
  expect(world.time).toBeCloseTo(0.01);
  stop();
  world.update(0);
  expect(observed).toEqual([0]);
  world.update(1);
  expect(world.time).toBeCloseTo(0.03);
  expect(observed).toEqual([0, 0.01, 0.02]);
});
