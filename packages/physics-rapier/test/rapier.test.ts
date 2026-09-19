import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh } from "three";
import {
  DistanceJoint,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { createWorld, inertialBody } from "./fixtures.js";
import { buildWorld } from "../src/index.js";

/** Engine limitations the adapter documents and rejects explicitly. */

for (const solverIterations of [0, -1, 1.5, NaN, Infinity]) {
  it(`rejects invalid solver iteration count ${solverIterations}`, async () => {
    await expect(buildWorld({ solverIterations })).rejects.toThrow(
      "solverIterations must be a positive integer",
    );
  });
}

it("rejects unequal static and dynamic friction", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  body.add(new Mesh(new BoxGeometry()));
  body.setMaterial({ staticFriction: 1, dynamicFriction: 0.2 });
  expect(() => world.update(world.fixedDelta)).toThrow("friction");
  body.setMaterial({});
  world.update(world.fixedDelta);
});

it("rejects distance joints with a positive minimum distance", async () => {
  const world = await createWorld();
  new DistanceJoint({
    body0: null,
    body1: inertialBody(),
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits: [0.5, 2],
  });
  expect(() => world.update(0)).toThrow("zero minimum distance");
});

it("rejects ambiguous revolute position goals before stepping, including huge finite targets", async () => {
  const world = await createWorld();
  const hinge = new RevoluteJoint({
    body0: null,
    body1: inertialBody(),
    axis: "Z",
  });
  const drive = new JointDrive({ stiffness: 100, damping: 10 });
  hinge.setDrive(drive);
  for (const position of [
    -Number.MAX_VALUE,
    -4 * Math.PI - 1,
    -Math.PI,
    Math.PI,
    4 * Math.PI + 1,
    Number.MAX_VALUE,
  ]) {
    drive.setTarget({ position });
    expect(() => world.update(0.01)).toThrow("within pi");
    expect(world.time).toBe(0);
    expect(hinge.getState().velocity).toBe(0);
  }
});
