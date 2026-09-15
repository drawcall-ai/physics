import { afterEach, beforeEach, expect, expectTypeOf, it } from "vitest";
import {
  AuthoringWorld,
  JointMotor,
  type JointMotorOptions,
  type JointMotorTarget,
  RevoluteJoint,
  RigidBody,
  setDefaultWorld,
} from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
  setDefaultWorld(world);
});
afterEach(() => world.dispose());
function hinge() {
  return new RevoluteJoint({ body0: null, body1: new RigidBody() });
}

it("authors one motor per axis joint without an implicit initial target", () => {
  const joint = hinge();
  const options = { joint, stiffness: 10, damping: 2, maxForce: 5 };
  const motor = new JointMotor(options);
  expect(joint.motor).toBe(motor);
  expect(motor.options).not.toBe(options);
  options.stiffness = 100;
  expect(motor.options.stiffness).toBe(10);
  expectTypeOf<Pick<JointMotor, "options">>().toEqualTypeOf<{
    readonly options: JointMotorOptions;
  }>();
  expect(motor.enabled).toBe(true);
  expect(motor.target).toBeUndefined();
  expect(motor.active).toBe(false);
  expect(world.objects.size).toBe(2);
  expect(() => new JointMotor({ joint })).toThrow("already has a motor");
  joint.setEffort(1);
});

it("replaces targets, supplies omitted zero coordinates and retains them while disabled", () => {
  const motor = new JointMotor({ joint: hinge(), damping: 2 });
  const target = { position: 1, velocity: 2 };
  motor.setTarget(target);
  target.position = 99;
  expect(motor.target).toEqual({ position: 1, velocity: 2 });
  expectTypeOf<Pick<JointMotor, "target">>().toEqualTypeOf<{
    readonly target: JointMotorTarget | undefined;
  }>();
  motor.setTarget({ velocity: 0 });
  expect(motor.target).toEqual({ position: 0, velocity: 0 });
  expect(motor.active).toBe(true);
  motor.setEnabled(false);
  expect(motor.active).toBe(false);
  expect(motor.target).toEqual({ position: 0, velocity: 0 });
  motor.setEnabled(true);
  expect(motor.active).toBe(true);
});

it("requires explicitly disabling an active motor before independent effort", () => {
  const joint = hinge();
  const motor = new JointMotor({ joint, damping: 2 }).setTarget({
    velocity: 1,
  });
  expect(() => joint.setEffort(2)).toThrow("Disable the joint motor");
  joint.setEffort(0);
  motor.setEnabled(false);
  joint.setEffort(2);
  expect(motor.target).toEqual({ position: 0, velocity: 1 });
});

it("validates immutable gains and finite targets without changing existing state", () => {
  for (const invalid of [
    { stiffness: -1 },
    { damping: NaN },
    { maxForce: Infinity },
  ]) {
    const joint = hinge();
    expect(() => new JointMotor({ joint, ...invalid })).toThrow(
      "finite and nonnegative",
    );
    expect(joint.motor).toBeUndefined();
  }
  const motor = new JointMotor({ joint: hinge() }).setTarget({ position: 1 });
  expectTypeOf<{}>().not.toExtend<Parameters<JointMotor["setTarget"]>[0]>();
  expect(() => motor.setTarget({ velocity: Infinity })).toThrow("finite");
  expect(motor.target).toEqual({ position: 1, velocity: 0 });
});

it("disposes motor actuation independently and with the joint/world", () => {
  const joint = hinge();
  const motor = new JointMotor({ joint });
  motor.dispose();
  motor.dispose();
  expect(joint.disposed).toBe(false);
  expect(joint.motor).toBeUndefined();
  expect(() => motor.setTarget({ velocity: 1 })).toThrow("disposed");
  expect(() => motor.setEnabled(true)).toThrow("disposed");
  const replacement = new JointMotor({ joint });
  joint.dispose();
  expect(replacement.disposed).toBe(true);
  expect(() => new JointMotor({ joint })).toThrow("disposed");
  const other = new JointMotor({ joint: hinge() });
  world.dispose();
  expect(other.disposed).toBe(true);
});
