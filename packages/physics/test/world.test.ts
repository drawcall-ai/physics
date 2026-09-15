import { afterEach, expect, it, vi } from "vitest";
import { Group, Matrix4, Vector3 } from "three";
import {
  AuthoringWorld,
  RigidBody,
  RevoluteJoint,
  setDefaultWorld,
  getDefaultWorld,
  clone,
} from "../src/index.js";

const worlds = new Set<AuthoringWorld>();
function setup() {
  const world = new AuthoringWorld();
  worlds.add(world);
  setDefaultWorld(world);
  return world;
}
afterEach(() => {
  for (const world of worlds) world.dispose();
  worlds.clear();
});

it("requires setup, captures the default, and accepts an explicit world", () => {
  expect(() => new RigidBody()).toThrow("setupWorld");
  const first = setup();
  const body = new RigidBody();
  const second = setup();
  const options = { world: first, mass: 20 };
  const explicit = new RigidBody(options);
  expect(explicit.options).not.toBe(options);
  expect(body.world).toBe(first);
  expect(first.objects.has(explicit)).toBe(true);
  expect(new RigidBody().world).toBe(second);
  first.dispose();
  expect(getDefaultWorld()).toBe(second);
  expect(body.disposed).toBe(true);
  second.dispose();
  expect(() => getDefaultWorld()).toThrow("setupWorld");
});

it("registers joints in their bodies' world and disposes connected joints", () => {
  const first = setup();
  const body0 = new RigidBody(),
    body1 = new RigidBody();
  setup();
  const options = { body0, body1 };
  const hinge = new RevoluteJoint(options);
  expect(hinge.options).not.toBe(options);
  expect(hinge.world).toBe(first);
  expect(first.objects.size).toBe(3);
  expect(() => new RevoluteJoint({ body0, body1: new RigidBody() })).toThrow(
    "same world",
  );
  body1.dispose();
  expect(hinge.disposed).toBe(true);
  expect(first.objects.size).toBe(1);
  body1.dispose();
  expect(() => new RevoluteJoint({ body0, body1 })).toThrow("disposed");
});

it("clones assemblies in their original world and remaps joint references", () => {
  const world = setup();
  const root = new Group();
  const body0 = new RigidBody(),
    body1 = new RigidBody();
  const hinge = new RevoluteJoint({
    body0,
    body1,
    limits: [0, 1],
  });
  root.add(hinge, body0, body1);
  const nextWorld = setup();
  const result = clone(root);
  const copy = result.children[0];
  if (!(copy instanceof RevoluteJoint))
    throw new Error("Expected a joint clone");
  expect(copy.options.body0).toBe(result.children[1]);
  expect(copy.options.body1).toBe(result.children[2]);
  expect(copy.limits).not.toBe(hinge.limits);
  expect(world.objects.size).toBe(6);
  expect(nextWorld.objects.size).toBe(0);
  const standalone = hinge.clone();
  expect(standalone.options.body0).toBe(body0);
  expect(standalone.options.body1).toBe(body1);
});

it("copies velocity tuples and rolls back when Three.js copy fails", () => {
  const world = setup();
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
  expect(world.objects.size).toBe(2);
});

it("preserves subclasses and cleans partially cloned assemblies", () => {
  const world = setup();
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
  expect(world.objects.size).toBe(4);
});

it("remaps joints across nested groups and retains references outside an assembly", () => {
  setup();
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
  setup();
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
  const world = setup();
  const body = new RigidBody();
  body.add(new RevoluteJoint({ body0: null, body1: body }));
  const copy = clone(body);
  const joint = copy.children[0];
  if (!(joint instanceof RevoluteJoint)) throw new Error("Expected a joint");
  expect(joint.options.body1).toBe(copy);
  expect(joint.disposed).toBe(false);
  expect(world.objects.size).toBe(4);
});

it("removes cloned registrations when a later child of a joint fails to clone", () => {
  const world = setup();
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const child = new RigidBody();
  const invalid = new Group();
  invalid.userData.self = invalid.userData;
  joint.add(child, invalid);
  expect(() => joint.clone()).toThrow();
  expect(world.objects.size).toBe(3);
});

it("registers each cloned joint once without unregistering it", () => {
  const world = setup();
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const root = new Group().add(joint, body);
  const register = vi.spyOn(world, "register");
  const unregister = vi.spyOn(world, "unregister");
  const result = clone(root);
  expect(register.mock.calls.map(([object]) => object)).toEqual([
    result.children[1],
    result.children[0],
  ]);
  expect(unregister).not.toHaveBeenCalled();
});

it("supports a static preview callback without a backend or step observers", () => {
  const world = setup();
  const body = new RigidBody().setType("kinematic");
  world.unregister(body);
  const before = vi.fn(),
    after = vi.fn();
  const stop = world.onBeforeStep(before);
  world.onAfterStep(after);
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.teleport(new Matrix4().makeTranslation(1, 2, 3));
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const onFrame = () => {
    body.updateWorldMatrix(true, false);
    expect(body.matrixWorld.elements[12]).toBe(1);
    expect(body.getVelocity().linear.x).toBe(2);
    expect(joint.getState().position).toBeCloseTo(0);
    body.applyImpulse(new Vector3(3, 0, 0));
    body.applyForce(new Vector3(1, 0, 0));
    body.sleep();
    body.wake();
    body.setKinematicTarget(new Matrix4().makeTranslation(9, 9, 9));
  };
  onFrame();
  stop();
  expect(body.position.toArray()).toEqual([1, 2, 3]);
  expect(body.getVelocity().linear.x).toBe(2);
  expect(before).not.toHaveBeenCalled();
  expect(after).not.toHaveBeenCalled();
  expect(() => world.update(world.fixedDelta)).toThrow("cannot simulate");
  expect(() => body.applyForce(new Vector3(NaN, 0, 0))).toThrow("finite");
  expect(() => body.teleport(new Matrix4().makeScale(2, 2, 2))).toThrow(
    "unit scale",
  );
  world.dispose();
  expect(() => body.getVelocity()).toThrow("disposed");
});

it("remaps bodies beneath a joint used as the hierarchy root", () => {
  setup();
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  joint.add(body);
  const copy = clone(joint);
  expect(copy.options.body1).toBe(copy.children[0]);
  expect(copy.options.body1).not.toBe(body);
});
