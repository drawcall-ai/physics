import { afterEach, beforeEach, expect, it } from "vitest";
import { Group, Vector3 } from "three";
import {
  AuthoringWorld,
  JointMotor,
  RevoluteJoint,
  RigidBody,
  clone,
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
  expect(Object.isFrozen(motor.options)).toBe(true);
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
  expect(Object.isFrozen(motor.target)).toBe(true);
  motor.setTarget({ velocity: 0 });
  expect(motor.target).toEqual({ position: 0, velocity: 0 });
  expect(motor.active).toBe(true);
  motor.setEnabled(false);
  expect(motor.active).toBe(false);
  expect(motor.target).toEqual({ position: 0, velocity: 0 });
  motor.setEnabled(true);
  expect(motor.active).toBe(true);
  expect(Reflect.set(motor, "enabled", false)).toBe(false);
  expect(Reflect.set(motor, "options", {})).toBe(false);
  expect(Reflect.set(motor, "target", {})).toBe(false);
});

it("requires explicitly disabling an active motor before independent effort", () => {
  const joint = hinge();
  const motor = new JointMotor({ joint, damping: 2 }).setTarget({
    velocity: 1,
  });
  expect(() => joint.setEffort(2)).toThrow("Disable the joint motor");
  expect(() => world.setJointEffort(joint, 2)).toThrow(
    "Disable the joint motor",
  );
  joint.setEffort(0);
  motor.setEnabled(false);
  joint.setEffort(2);
  expect(motor.target).toEqual({ position: 0, velocity: 1 });
});

it("validates immutable gains and finite targets without changing existing state", () => {
  for (const value of [-1, NaN, Infinity]) {
    const joint = hinge();
    expect(() => new JointMotor({ joint, stiffness: value })).toThrow(
      "finite and nonnegative",
    );
    expect(() => new JointMotor({ joint, damping: value })).toThrow(
      "finite and nonnegative",
    );
    expect(() => new JointMotor({ joint, maxForce: value })).toThrow(
      "finite and nonnegative",
    );
    expect(joint.motor).toBeUndefined();
  }
  const motor = new JointMotor({ joint: hinge() }).setTarget({ position: 1 });
  expect(() => motor.setTarget({})).toThrow("requires position or velocity");
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

it("clones motor configuration and target state while rebinding the copied joint", () => {
  const joint = hinge();
  const motor = new JointMotor({
    joint,
    stiffness: 3,
    damping: 2,
    maxForce: 4,
    model: "acceleration",
  })
    .setTarget({ position: 1 })
    .setEnabled(false);
  joint.copy(joint);
  expect(joint.motor).toBe(motor);
  const root = new Group().add(joint, joint.options.body1);
  const copy = clone(root).children[0];
  if (!(copy instanceof RevoluteJoint)) throw new Error("Missing copied joint");
  const copiedMotor = copy.motor;
  if (!copiedMotor) throw new Error("Missing copied motor");
  expect(copiedMotor).not.toBe(motor);
  expect(copiedMotor.options).toEqual({ ...motor.options, joint: copy });
  expect(copiedMotor.target).toEqual(motor.target);
  expect(copiedMotor.enabled).toBe(false);
  copiedMotor.setTarget({ velocity: 5 });
  expect(motor.target).toEqual({ position: 1, velocity: 0 });
  copy.dispose();
  expect(copiedMotor.disposed).toBe(true);
  expect(motor.disposed).toBe(false);
});

it("clears authored velocity on actual body type changes and preserves it on no-op changes", () => {
  const body = new RigidBody().setVelocity({
    linear: new Vector3(1, 2, 3),
    angular: new Vector3(4, 5, 6),
  });
  expect(body.bodyType).toBe("dynamic");
  body.setType("dynamic");
  expect(body.getVelocity().linear.toArray()).toEqual([1, 2, 3]);
  body.setType("kinematic");
  expect(body.getVelocity().linear.length()).toBe(0);
  expect(body.getVelocity().angular.length()).toBe(0);
  body.setVelocity({ linear: new Vector3(3, 0, 0) });
  expect(body.clone().bodyType).toBe("kinematic");
  expect(body.clone().getVelocity().linear.x).toBe(3);
  expect(Reflect.set(body, "bodyType", "static")).toBe(false);
  body.dispose();
  expect(() => body.setType("static")).toThrow("disposed");
});
