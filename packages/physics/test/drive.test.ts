import { afterEach, expect, expectTypeOf, it } from "vitest";
import {
  registry,
  DistanceJoint,
  GenericJoint,
  JointDrive,
  type JointDriveOptions,
  type JointDriveTarget,
  RevoluteJoint,
  RigidBody,
} from "../src/index.js";

afterEach(() => registry.clear());
function hinge() {
  return new RevoluteJoint({ body0: null, body1: new RigidBody() });
}

it("freezes drive options and starts passive", () => {
  const options = { stiffness: 10, damping: 2, maxForce: 5 };
  const drive = new JointDrive(options);
  expect(drive.options).not.toBe(options);
  options.stiffness = 100;
  expect(drive.options.stiffness).toBe(10);
  expectTypeOf<Pick<JointDrive, "options">>().toEqualTypeOf<{
    readonly options: JointDriveOptions;
  }>();
  expect(drive.target).toBeUndefined();
  expect(drive.joint).toBeUndefined();
  for (const invalid of [
    { stiffness: -1 },
    { damping: NaN },
    { maxForce: Infinity },
  ])
    expect(() => new JointDrive(invalid)).toThrow("finite and nonnegative");
});

it("replaces the whole target, zeroes omitted terms, and requires the gain a term acts through", () => {
  const drive = new JointDrive({ damping: 2 });
  const target = { velocity: 2, effort: 1 };
  drive.setTarget(target);
  target.velocity = 99;
  expect(drive.target).toEqual({ position: 0, velocity: 2, effort: 1 });
  expectTypeOf<Pick<JointDrive, "target">>().toEqualTypeOf<{
    readonly target: JointDriveTarget | undefined;
  }>();
  drive.setTarget({ effort: 3 });
  expect(drive.target).toEqual({ position: 0, velocity: 0, effort: 3 });
  expect(() => drive.setTarget({ position: 1 })).toThrow("needs stiffness");
  expect(() => new JointDrive({}).setTarget({ velocity: 1 })).toThrow(
    "needs damping",
  );
  expect(() => drive.setTarget({ effort: NaN })).toThrow("finite");
  drive.setTarget(undefined);
  expect(drive.target).toBeUndefined();
});

it("attaches to one joint at a time and detaches on replacement and disposal", () => {
  const first = hinge(),
    second = hinge();
  const drive = new JointDrive({ stiffness: 1 });
  first.setDrive(drive);
  expect(drive.joint).toBe(first);
  expect(first.drive).toBe(drive);
  expect(() => second.setDrive(drive)).toThrow("already attached");
  first.setDrive(drive);
  const replacement = new JointDrive({ damping: 1 });
  first.setDrive(replacement);
  expect(drive.joint).toBeUndefined();
  expect(replacement.joint).toBe(first);
  second.setDrive(drive);
  expect(drive.joint).toBe(second);
  first.dispose();
  expect(replacement.joint).toBeUndefined();
  expect(() => first.setDrive(undefined)).toThrow("disposed");
});

it("clones drives with their subclass and copies them with their joint", () => {
  class Actuator extends JointDrive<
    JointDriveOptions & { interface: string }
  > {}
  const joint = hinge();
  const actuator = new Actuator({
    stiffness: 4,
    interface: "position",
  }).setTarget({
    position: 0.5,
  });
  joint.setDrive(actuator);
  const copy = joint.clone();
  expect(copy.drive).toBeInstanceOf(Actuator);
  expect(copy.drive?.options).toEqual(actuator.options);
  expect(copy.drive?.target).toEqual(actuator.target);
  expect(actuator.joint).toBe(joint);
  const distance = new DistanceJoint({
    body0: null,
    body1: new RigidBody(),
    limits: [0, Infinity],
  }).setDrive(new JointDrive({ stiffness: 10 }).setTarget({ position: 0 }));
  expect(distance.clone().drive?.options.stiffness).toBe(10);
});

it("locks generic joint axes by default and drives each axis separately", () => {
  const body = new RigidBody();
  const joint = new GenericJoint({
    body0: null,
    body1: body,
    dofs: { transY: "free", rotX: [-1, 1] },
  });
  expect(joint.dofs).toEqual({
    transX: "locked",
    transY: "free",
    transZ: "locked",
    rotX: [-1, 1],
    rotY: "locked",
    rotZ: "locked",
  });
  const lift = new JointDrive({ stiffness: 100 }).setTarget({ position: 1 });
  joint.setDrive("transY", lift);
  expect(joint.getDrive("transY")).toBe(lift);
  expect(() => joint.setDrive("rotX", lift)).toThrow("already attached");
  expect(joint.drives.size).toBe(1);
  expect(joint.clone().getDrive("transY")?.target).toEqual(lift.target);
  expect(joint.getState("transY")).toEqual({ position: 0, velocity: 0 });
  joint.setDrive("transY", undefined);
  expect(lift.joint).toBeUndefined();
  expect(
    () =>
      new GenericJoint({ body0: null, body1: body, dofs: { rotZ: [2, 1] } }),
  ).toThrow("Invalid joint limits");
  expect(() =>
    joint.copy(new GenericJoint({ body0: null, body1: body })),
  ).toThrow("immutable");
});
