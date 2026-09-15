import { afterEach, beforeEach, expect, it } from "vitest";
import { Matrix4, Vector3 } from "three";
import {
  AuthoringWorld,
  BoxCollider,
  DistanceJoint,
  FixedJoint,
  MeshCollider,
  PrismaticJoint,
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

it("captures immutable tuples and frames while preserving body and world identities", () => {
  const centerOfMass: [number, number, number] = [1, 2, 3];
  const diagonalInertia: [number, number, number] = [2, 3, 4];
  const principalAxes: [number, number, number, number] = [0, 0, 0, 1];
  const options = {
    world,
    mass: 2,
    centerOfMass,
    diagonalInertia,
    principalAxes,
  };
  const body = new RigidBody(options);
  options.mass = 20;
  centerOfMass[0] = 9;
  diagonalInertia[1] = 99;
  principalAxes[3] = 0;
  expect(body.options.mass).toBe(2);
  expect(body.options.centerOfMass).toEqual([1, 2, 3]);
  expect(body.options.diagonalInertia).toEqual([2, 3, 4]);
  expect(body.options.principalAxes).toEqual([0, 0, 0, 1]);
  expect(Object.isFrozen(body.options)).toBe(true);
  expect(Object.isFrozen(body.options.centerOfMass)).toBe(true);
  expect(body.options.world).toBe(world);
  const frame0 = new Matrix4().makeTranslation(2, 0, 0);
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
    frame0,
    frame1: new Matrix4(),
  });
  frame0.makeTranslation(10, 0, 0);
  joint.options.frame0?.makeTranslation(20, 0, 0);
  expect(joint.getFrame(0, new Matrix4()).elements[12]).toBe(2);
  expect(joint.options.body1).toBe(body);
});

it("validates controls before storing and clones independent readable settings", () => {
  const material = { density: 12, restitution: 0.4 };
  const groups = { membership: 2, filter: 3 };
  const size: [number, number, number] = [1, 2, 3];
  const collider = new BoxCollider({ size })
    .setMaterial(material)
    .setCollisionGroups(groups)
    .setSensor(true);
  const copy = collider.clone();
  material.density = 99;
  groups.filter = 0;
  size[0] = 9;
  expect(collider.size).toEqual([1, 2, 3]);
  expect(collider.material?.density).toBe(12);
  expect(collider.collisionGroups?.filter).toBe(3);
  expect(Object.isFrozen(collider.material)).toBe(true);
  copy.setSensor(false).setMaterial({ density: 5 });
  expect(collider.sensor).toBe(true);
  expect(collider.material?.density).toBe(12);
  expect(() => collider.setMaterial({ restitution: 2 })).toThrow("material");
  expect(() => new BoxCollider({ size: [1, 0, 1] })).toThrow("positive");
  expect(collider.size).toEqual([1, 2, 3]);
  const body = new RigidBody()
    .setLinearDamping(2)
    .setAngularDamping(3)
    .setGravityScale(-1);
  expect(() => body.setLinearDamping(-1)).toThrow("nonnegative");
  expect(() => body.setGravityScale(Infinity)).toThrow("finite");
  expect(body.linearDamping).toBe(2);
  expect(body.gravityScale).toBe(-1);
  body.dispose();
  expect(() => body.setAngularDamping(0)).toThrow("disposed");
});

it("validates physical mass specifications before registration", () => {
  expect(() => new RigidBody({ mass: 0 })).toThrow("mass");
  expect(
    () =>
      new RigidBody({
        mass: 1,
        centerOfMass: [0, 0, 0],
        diagonalInertia: [1, 1, 3],
      }),
  ).toThrow("triangle");
  expect(
    () =>
      new RigidBody({
        mass: 1,
        centerOfMass: [0, 0, 0],
        diagonalInertia: [0, 1, 1],
      }),
  ).toThrow("positive");
  expect(
    () =>
      new RigidBody({
        mass: 1,
        centerOfMass: [0, 0, 0],
        diagonalInertia: [1, 1, 1],
        principalAxes: [0, 0, 0, 2],
      }),
  ).toThrow("normalized");
  expect(world.objects.size).toBe(0);
  const body = new RigidBody({
    mass: 2,
    diagonalInertia: [1, 1, 1],
    centerOfMass: [0, 0, 0],
    colliders: false,
  });
  expect(body.getColliders()).toEqual([]);
  expect(body.clone().options.diagonalInertia).toEqual([1, 1, 1]);
});

it("compares immutable copy values independently of option order and explicit defaults", () => {
  const source = new RigidBody({ mass: 2 });
  const target = new RigidBody({
    mass: 2,
    colliders: "auto",
    canSleep: true,
  });
  source
    .setVelocity({ linear: new Vector3(2, 0, 0) })
    .setMaterial({ density: 12 });
  target.copy(source);
  expect(target.getVelocity().linear.x).toBe(2);
  expect(target.material).toEqual({ density: 12 });
  expect(() => new RigidBody({ mass: 3 }).copy(source)).toThrow("immutable");
  const mesh = new MeshCollider({ approximation: "trimesh" });
  expect(mesh.clone().approximation).toBe("trimesh");
  expect(mesh.clone().geometry).toBe(mesh.geometry);
});

it("keeps construction controls readable and validates authoring effort without simulation", () => {
  const body = new RigidBody();
  const limits: [number, number] = [-1, 2];
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
    limits,
  }).setCollideConnected(true);
  limits[0] = -99;
  expect(joint.limits).toEqual([-1, 2]);
  joint.setEffort(2).setEffort(3).setEnabled(false).setEnabled(true);
  expect(joint.enabled).toBe(true);
  expect(joint.collideConnected).toBe(true);
  expect(joint.getState()).toEqual({ position: 0, velocity: 0 });
  expect(() => joint.setEffort(NaN)).toThrow("finite");
  expect(
    new RevoluteJoint({ body0: null, body1: body }).limits,
  ).toBeUndefined();
  expect(
    () => new DistanceJoint({ body0: null, body1: body, limits: [-1, 2] }),
  ).toThrow("nonnegative");
  const foreign = new AuthoringWorld();
  expect(() => foreign.setJointEffort(joint, 2)).toThrow("another world");
  foreign.dispose();
  expect(world.time).toBe(0);
  expect(() => world.raycast(new Vector3(), new Vector3(1, 0, 0), 10)).toThrow(
    "raycast",
  );
  joint.dispose();
  expect(() => joint.setEffort(1)).toThrow("disposed");
});

it("measures prismatic anchor velocity relative to the rotating reference axis", () => {
  const body0 = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  }).setVelocity({
    angular: new Vector3(0, 0, 2),
  });
  const body1 = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  }).setVelocity({
    linear: new Vector3(3, 0, 0),
    angular: new Vector3(0, 0, 4),
  });
  const joint = new PrismaticJoint({
    body0,
    body1,
    axis: "X",
    frame0: new Matrix4().makeTranslation(0, 1, 0),
    frame1: new Matrix4().makeTranslation(0, 3, 0),
  });
  // Anchor velocities are -2 and 3-12 along X; axis rotation contributes 2*2.
  expect(joint.getState()).toEqual({ position: 0, velocity: -3 });
  const fixed = new FixedJoint({
    body0: null,
    body1,
    frame0: new Matrix4(),
    frame1: new Matrix4().makeTranslation(0, 3, 0),
  });
  expect(fixed.getState().translation.toArray()).toEqual([0, 3, 0]);
});

it("enforces immutable properties for JavaScript callers", () => {
  const body = new RigidBody();
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const mesh = new MeshCollider();
  expect(Reflect.set(body, "options", {})).toBe(false);
  expect(Reflect.set(body, "world", world)).toBe(false);
  expect(Reflect.set(joint, "options", {})).toBe(false);
  expect(Reflect.set(joint, "world", world)).toBe(false);
  expect(Reflect.set(mesh, "approximation", "trimesh")).toBe(false);
});

it("reads non-prismatic state without inspecting unfinished visual geometry", () => {
  const body = new RigidBody().setVelocity({ angular: new Vector3(1, 2, 3) });
  const mesh = new MeshCollider();
  body.add(mesh);
  const hinge = new RevoluteJoint({ body0: null, body1: body });
  expect(hinge.getState().velocity).toBeCloseTo(2);
  expect(
    new FixedJoint({ body0: null, body1: body })
      .getState()
      .translation.length(),
  ).toBe(0);
});

it("honors subclass copy overrides for standalone joint cloning", () => {
  class Hinge extends RevoluteJoint {
    label = "";
    override copy(source: this, recursive = true): this {
      super.copy(source, recursive);
      this.label = source.label;
      return this;
    }
  }
  const hinge = new Hinge({ body0: null, body1: new RigidBody() });
  hinge.label = "door";
  expect(hinge.clone().label).toBe("door");
});

it("keeps authoring reads explicit when rotating slider velocity needs inferred mass", () => {
  const body = new RigidBody().setVelocity({ angular: new Vector3(0, 0, 2) });
  const slider = new PrismaticJoint({ body0: null, body1: body });
  expect(() => slider.getState()).toThrow("complete explicit mass properties");
  expect(body.getVelocity().angular.z).toBe(2);
  body.setVelocity({ angular: new Vector3(), linear: new Vector3(0, 3, 0) });
  expect(slider.getState().position).toBe(0);
  expect(slider.getState().velocity).toBeCloseTo(3);
});

it("captures immutable joint limits and rejects incompatible copies", () => {
  const body = new RigidBody();
  const limits: [number, number] = [-1, 2];
  const hinge = new RevoluteJoint({ body0: null, body1: body, limits });
  limits[0] = -99;
  expect(hinge.options.limits).toEqual([-1, 2]);
  expect(Object.isFrozen(hinge.limits)).toBe(true);
  expect(Reflect.set(hinge, "limits", [-3, 3])).toBe(false);
  const copy = hinge.clone();
  expect(copy.limits).toEqual([-1, 2]);
  expect(copy.limits).not.toBe(hinge.limits);
  expect(() =>
    new RevoluteJoint({ body0: null, body1: body }).copy(hinge),
  ).toThrow("immutable");
  const distance = new DistanceJoint({
    body0: null,
    body1: body,
    limits: [1, 3],
  });
  expect(distance.clone().options.limits).toEqual([1, 3]);
  expect(() =>
    new DistanceJoint({ body0: null, body1: body }).copy(distance),
  ).toThrow("immutable");
});

it("accepts total mass or complete explicit mass properties and rejects partial overrides", () => {
  expect(new RigidBody({ mass: 2 }).options.mass).toBe(2);
  expect(() => new RigidBody({ centerOfMass: [0, 0, 0] })).toThrow("together");
  expect(() => new RigidBody({ mass: 2, centerOfMass: [0, 0, 0] })).toThrow(
    "together",
  );
  expect(() => new RigidBody({ mass: 2, diagonalInertia: [1, 1, 1] })).toThrow(
    "together",
  );
  expect(() => new RigidBody({ mass: 2, principalAxes: [0, 0, 0, 1] })).toThrow(
    "together",
  );
  const complete = new RigidBody({
    mass: 2,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  expect(() =>
    new RigidBody({ ...complete.options, principalAxes: [0, 0, 0, 1] }).copy(
      complete,
    ),
  ).not.toThrow();
});
