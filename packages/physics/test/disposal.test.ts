import { expect, it } from "vitest";
import { Group } from "three";
import {
  registry,
  GenericJoint,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "../src/index.js";

it("marks joints disposed and releases all drive slots before removed callbacks", () => {
  const body = new RigidBody({});
  const first = new JointDrive({});
  const second = new JointDrive({});
  const joint = new GenericJoint({ body0: null, body1: body })
    .setDrive("transX", first)
    .setDrive("rotY", second);
  new Group().add(joint);
  const error = new Error("removed callback failed");
  joint.addEventListener("removed", () => {
    expect(joint.disposed).toBe(true);
    expect(first.joint).toBeUndefined();
    expect(second.joint).toBeUndefined();
    joint.dispose();
    throw error;
  });
  expect(() => joint.dispose()).toThrow(error);
  expect(joint.drives.size).toBe(0);
  expect(joint.parent).toBeNull();
  expect(registry.objects.has(joint)).toBe(false);
  registry.clear();
});

it("cleans authoring registrations and the default world after multiple scene errors", () => {
  const body = new RigidBody();
  const drive = new JointDrive({});
  const first = new RevoluteJoint({ body0: null, body1: body }).setDrive(drive);
  const second = new RevoluteJoint({ body0: null, body1: body });
  new Group().add(body, first, second);
  const firstError = new Error("first callback failed");
  const secondError = new Error("second callback failed");
  first.addEventListener("removed", () => {
    throw firstError;
  });
  second.addEventListener("removed", () => {
    throw secondError;
  });
  let failure: unknown;
  try {
    registry.clear();
  } catch (error) {
    failure = error;
  }
  expect(failure).toBeInstanceOf(AggregateError);
  if (!(failure instanceof AggregateError))
    throw new Error("Expected disposal errors");
  expect(failure.errors).toEqual([firstError, secondError]);
  expect(registry.objects.size).toBe(0);
  expect(drive.joint).toBeUndefined();
  for (const object of [first, second, body]) {
    expect(object.disposed).toBe(true);
    expect(object.parent).toBeNull();
  }
  expect(() => registry.requireWorld()).toThrow();
  expect(() => registry.clear()).not.toThrow();
});
