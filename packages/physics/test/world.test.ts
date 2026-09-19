import { afterEach, expect, it, vi } from "vitest";
import { Group, Matrix4, Vector3 } from "three";
import {
  registry,
  JointDrive,
  RigidBody,
  RevoluteJoint,
  clone,
} from "../src/index.js";

afterEach(() => {
  registry.clear();
  vi.restoreAllMocks();
});

it("registers objects before a world exists and disposes connected joints", () => {
  const body0 = new RigidBody(),
    body1 = new RigidBody();
  const joint = new RevoluteJoint({ body0, body1 });
  expect(registry.objects.size).toBe(3);
  expect(registry.world).toBeUndefined();
  body1.dispose();
  expect(joint.disposed).toBe(true);
  expect(registry.objects.size).toBe(1);
  expect(() => new RevoluteJoint({ body0, body1 })).toThrow("disposed");
  registry.clear();
});

it("clones registered assemblies and remaps joint references", () => {
  const root = new Group();
  const body0 = new RigidBody(),
    body1 = new RigidBody();
  const hinge = new RevoluteJoint({
    body0,
    body1,
    limits: [0, 1],
  });
  const drive = new JointDrive({
    stiffness: 3,
    damping: 2,
    maxForce: 4,
    model: "acceleration",
  }).setTarget({ position: 1 });
  hinge.setDrive(drive);
  hinge.copy(hinge);
  expect(hinge.drive).toBe(drive);
  root.add(hinge, body0, body1);
  const result = clone(root);
  const copy = result.children[0];
  if (!(copy instanceof RevoluteJoint))
    throw new Error("Expected a joint clone");
  expect(copy.options.body0).toBe(result.children[1]);
  expect(copy.options.body1).toBe(result.children[2]);
  expect(copy.limits).not.toBe(hinge.limits);
  const copiedDrive = copy.drive;
  if (!copiedDrive) throw new Error("Missing copied drive");
  expect(copiedDrive).not.toBe(drive);
  expect(copiedDrive.joint).toBe(copy);
  expect(copiedDrive.options).toEqual(drive.options);
  expect(copiedDrive.target).toEqual(drive.target);
  copiedDrive.setTarget({ velocity: 5 });
  expect(drive.target).toEqual({ position: 1, velocity: 0, effort: 0 });
  expect(registry.objects.size).toBe(6);
  const standalone = hinge.clone();
  expect(standalone.options.body0).toBe(body0);
  expect(standalone.options.body1).toBe(body1);
});

it("copies velocity tuples and rolls back when Three.js copy fails", () => {
  const source = new RigidBody().setVelocity({
    linear: new Vector3(1, 2, 3),
    angular: new Vector3(4, 5, 6),
  });
  const copy = clone(source);
  expect(copy.getVelocity()).toEqual(source.getVelocity());
  copy.setVelocity({ linear: new Vector3(9, 9, 9) });
  expect(source.getVelocity().linear.toArray()).toEqual([1, 2, 3]);
  source.userData.self = source.userData;
  expect(() => clone(source)).toThrow();
  expect(registry.objects.size).toBe(2);
});

it("preserves subclasses and cleans partially cloned assemblies", () => {
  class Mechanism extends Group {}
  class Body extends RigidBody {}
  const source = new Mechanism();
  const first = new Body(),
    second = new Body();
  source.add(first, second);
  const copy = clone(source);
  expect(copy).toBeInstanceOf(Mechanism);
  expect(copy.children[0]).toBeInstanceOf(Body);
  second.userData.self = second.userData;
  expect(() => clone(source)).toThrow();
  expect(registry.objects.size).toBe(4);
});

it("remaps joints across nested groups and retains references outside an assembly", () => {
  const source = new Group();
  const external = new RigidBody(),
    internal = new RigidBody();
  const group = new Group();
  const hinge = new RevoluteJoint({ body0: external, body1: internal });
  group.add(internal);
  source.add(group, hinge);
  const copy = clone(source);
  const joint = copy.children[1];
  if (!(joint instanceof RevoluteJoint)) throw new Error("Expected a joint");
  expect(joint.options.body0).toBe(external);
  expect(joint.options.body1).toBe(copy.children[0]?.children[0]);
});

it("native Group.copy retains the joint references", () => {
  const source = new Group();
  const body = new RigidBody();
  source.add(body, new RevoluteJoint({ body0: null, body1: body }));
  const target = new Group();
  const existing = new RigidBody();
  target.add(existing);
  target.copy(source);
  const joint = target.children[2];
  if (!(joint instanceof RevoluteJoint)) throw new Error("Expected a joint");
  expect(target.children[0]).toBe(existing);
  expect(joint.options.body1).toBe(body);
});

it("clones a body with a child joint and keeps the remapped joint registered", () => {
  const body = new RigidBody();
  body.add(new RevoluteJoint({ body0: null, body1: body }));
  const copy = clone(body);
  const joint = copy.children[0];
  if (!(joint instanceof RevoluteJoint)) throw new Error("Expected a joint");
  expect(joint.options.body1).toBe(copy);
  expect(joint.disposed).toBe(false);
  expect(registry.objects.size).toBe(4);
});

it("removes cloned registrations when a later child of a joint fails to clone", () => {
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const child = new RigidBody();
  const invalid = new Group();
  invalid.userData.self = invalid.userData;
  joint.add(child, invalid);
  expect(() => joint.clone()).toThrow();
  expect(registry.objects.size).toBe(3);
});

it("registers each cloned joint once without unregistering it", () => {
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const root = new Group().add(joint, body);
  const register = vi.spyOn(registry, "register");
  const unregister = vi.spyOn(registry, "unregister");
  const result = clone(root);
  expect(register.mock.calls.map(([object]) => object)).toEqual([
    result.children[1],
    result.children[0],
  ]);
  expect(unregister).not.toHaveBeenCalled();
});

it("authors poses and velocities before building and rejects simulation commands", () => {
  const body = new RigidBody({ type: "kinematic" });
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.teleport(new Matrix4().makeTranslation(1, 2, 3));
  expect(body.position.toArray()).toEqual([1, 2, 3]);
  expect(body.getVelocity().linear.x).toBe(2);
  const joint = new RevoluteJoint({ body0: null, body1: body });
  expect(joint.getState().position).toBeCloseTo(0);
  expect(() => body.applyImpulse(new Vector3(3, 0, 0))).toThrow("buildWorld");
  expect(() => body.setKinematicTarget(new Matrix4())).toThrow("buildWorld");
  expect(() => body.applyForce(new Vector3(NaN, 0, 0))).toThrow("finite");
  expect(() => body.teleport(new Matrix4().makeScale(2, 2, 2))).toThrow(
    "unit scale",
  );
  body.dispose();
  expect(() => body.getVelocity()).toThrow("disposed");
});

it("remaps bodies beneath a joint used as the hierarchy root", () => {
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  joint.add(body);
  const copy = clone(joint);
  expect(copy.options.body1).toBe(copy.children[0]);
  expect(copy.options.body1).not.toBe(body);
});
