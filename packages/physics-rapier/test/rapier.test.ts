import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh } from "three";
import {
  DistanceJoint,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { createWorld, inertialBody } from "./fixtures.js";

/** Engine limitations the adapter documents and rejects explicitly. */

it("rejects drives Rapier cannot model", async () => {
  const world = await createWorld();
  const body = inertialBody();
  world.scene.add(body);
  const joint = new RevoluteJoint({ body0: null, body1: body });
  world.scene.add(joint);
  joint.setDrive(
    new JointDrive({ stiffness: 10, maxForce: 1, maxVelocity: 1.5 }).setTarget({
      position: 1,
    }),
  );
  expect(() => world.update(0)).toThrow("cannot model maxVelocity");
  joint.setDrive(new JointDrive({ armature: 0.1 }));
  expect(() => world.update(0)).toThrow("no friction or armature");
  joint.setDrive(new JointDrive({ frictionLoss: 1 }));
  expect(() => world.update(0)).toThrow("no friction or armature");
});

it("rejects friction Rapier cannot model", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  body.add(new Mesh(new BoxGeometry()));
  world.scene.add(body);
  body.setMaterial({ staticFriction: 1, dynamicFriction: 0.2 });
  expect(() => world.update(world.fixedDelta)).toThrow("friction");
  body.setMaterial({ rollingFriction: 0.01 });
  expect(() => world.update(world.fixedDelta)).toThrow(
    "no torsional or rolling friction",
  );
  body.setMaterial({ torsionalFriction: 0.01 });
  expect(() => world.update(world.fixedDelta)).toThrow(
    "no torsional or rolling friction",
  );
  body.setMaterial({});
  world.update(world.fixedDelta);
});

it("rejects distance joints with a positive minimum distance", async () => {
  const world = await createWorld();
  const body = inertialBody();
  world.scene.add(body);
  world.scene.add(
    new DistanceJoint({
      body0: null,
      body1: body,
      frame0: new Matrix4(),
      frame1: new Matrix4(),
      limits: [0.5, 2],
    }),
  );
  expect(() => world.update(0)).toThrow("zero minimum distance");
});

it("rejects ambiguous revolute position goals before stepping, including huge finite targets", async () => {
  const world = await createWorld();
  const body = inertialBody();
  world.scene.add(body);
  const hinge = new RevoluteJoint({
    body0: null,
    body1: body,
    axis: "Z",
  });
  world.scene.add(hinge);
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
    // Reading the state joins the hinge first, which fails the same way.
    expect(() => hinge.getState()).toThrow("within pi");
  }
});
