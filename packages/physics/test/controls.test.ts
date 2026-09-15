import { afterEach, beforeEach, expect, expectTypeOf, it } from "vitest";
import { Matrix4, Vector3 } from "three";
import {
  AuthoringWorld,
  DistanceJoint,
  FixedJoint,
  MeshCollider,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
  setDefaultWorld,
  type RigidBodyOptions,
  type JointOptions,
  type Vec3,
} from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
  setDefaultWorld(world);
});
afterEach(() => world.dispose());
const mass = {
  mass: 1,
  centerOfMass: [0, 0, 0],
  diagonalInertia: [1, 1, 1],
} satisfies RigidBodyOptions;

it("copies immutable configuration, retaining resource identities and independent joint frames", () => {
  const centerOfMass: [number, number, number] = [1, 2, 3];
  const options = { ...mass, world, centerOfMass };
  const body = new RigidBody(options);
  expectTypeOf<
    Pick<RigidBody, "world" | "options" | "bodyType">
  >().toEqualTypeOf<
    Readonly<Pick<RigidBody, "world" | "options" | "bodyType">>
  >();
  options.mass = 20;
  centerOfMass[0] = 9;
  expect(body.options).toEqual({ ...mass, world, centerOfMass: [1, 2, 3] });
  const frame0 = new Matrix4().makeTranslation(2, 0, 0);
  const limits: [number, number] = [-1, 2];
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
    frame0,
    frame1: new Matrix4(),
    limits,
  });
  frame0.makeTranslation(10, 0, 0);
  joint.options.frame0?.makeTranslation(20, 0, 0);
  limits[0] = -99;
  expect(joint.getFrame(0, new Matrix4()).elements[12]).toBe(2);
  expect(joint.options.body1).toBe(body);
  expect(joint.clone().limits).toEqual([-1, 2]);
  expect(() =>
    new RevoluteJoint({ body0: null, body1: body }).copy(joint),
  ).toThrow("immutable");
  const distance = new DistanceJoint({
    body0: null,
    body1: body,
    limits: [1, 3],
  });
  expect(distance.clone().limits).toEqual([1, 3]);
  expect(() =>
    new DistanceJoint({ body0: null, body1: body }).copy(distance),
  ).toThrow("immutable");
});

const invalidMass: [RigidBodyOptions, string][] = [
  [{ mass: 0 }, "mass"],
  [{ ...mass, diagonalInertia: [1, 1, 3] satisfies Vec3 }, "triangle"],
  [{ ...mass, diagonalInertia: [0, 1, 1] satisfies Vec3 }, "positive"],
  [
    {
      ...mass,
      principalAxes: [0, 0, 0, 2] satisfies readonly [
        number,
        number,
        number,
        number,
      ],
    },
    "normalized",
  ],
];
it.each(invalidMass)(
  "rejects invalid mass properties before registration: %j",
  (options, error) => {
    expect(() => new RigidBody(options)).toThrow(error);
    expect(world.objects.size).toBe(0);
  },
);

it("copies matching immutable configurations regardless of option order and explicit defaults", () => {
  const source = new RigidBody(mass).setVelocity({
    linear: new Vector3(2, 0, 0),
  });
  const target = new RigidBody({
    canSleep: true,
    colliders: "auto",
    ...mass,
    principalAxes: [0, 0, 0, 1],
  });
  target.copy(source);
  expect(target.getVelocity().linear.x).toBe(2);
  expect(source.getColliders()).toEqual([]);
  expect(() => new RigidBody({ ...mass, mass: 3 }).copy(source)).toThrow(
    "immutable",
  );
  const mesh = new MeshCollider({ approximation: "trimesh" });
  expect(mesh.clone().approximation).toBe("trimesh");
  expect(mesh.clone().geometry).toBe(mesh.geometry);
});

it("validates runtime controls before storing and checks the authoring world boundary", () => {
  const body = new RigidBody().setLinearDamping(2).setGravityScale(-1);
  expect(() => body.setLinearDamping(-1)).toThrow("nonnegative");
  expect(() => body.setGravityScale(Infinity)).toThrow("finite");
  expect(body.linearDamping).toBe(2);
  expect(body.gravityScale).toBe(-1);
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
  }).setCollideConnected(true);
  joint.setEffort(2).setEnabled(false).setEnabled(true);
  expect(joint.getState()).toEqual({ position: 0, velocity: 0 });
  expect(() => joint.setEffort(NaN)).toThrow("finite");
  const foreign = new AuthoringWorld();
  expect(() => foreign.setJointEffort(joint, 2)).toThrow("another world");
  foreign.dispose();
  expect(world.time).toBe(0);
  expect(() => world.raycast(new Vector3(), new Vector3(1, 0, 0), 10)).toThrow(
    "raycast",
  );
  body.dispose();
  expect(() => joint.setEffort(1)).toThrow("disposed");
  expect(() => body.setAngularDamping(0)).toThrow("disposed");
});

it("measures prismatic anchor velocity relative to the rotating reference axis", () => {
  const body0 = new RigidBody(mass).setVelocity({
    angular: new Vector3(0, 0, 2),
  });
  const body1 = new RigidBody(mass).setVelocity({
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
});

it("reads unfinished joints without inferring collider mass", () => {
  const body = new RigidBody().setVelocity({ angular: new Vector3(0, 2, 0) });
  body.add(new MeshCollider());
  expect(
    new RevoluteJoint({ body0: null, body1: body }).getState().velocity,
  ).toBeCloseTo(2);
  expect(new FixedJoint({ body0: null, body1: body }).getState().distance).toBe(
    0,
  );
  const slider = new PrismaticJoint({ body0: null, body1: body });
  expect(() => slider.getState()).toThrow("requires explicit mass properties");
  body.setVelocity({ angular: new Vector3(), linear: new Vector3(0, 3, 0) });
  expect(slider.getState().velocity).toBeCloseTo(3);
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

it("copies immutable body type and clones independent velocity", () => {
  const options: { type: "kinematic" | "static" } = { type: "kinematic" };
  const body = new RigidBody(options).setVelocity({
    linear: new Vector3(3, 0, 0),
  });
  options.type = "static";
  expect(body.bodyType).toBe("kinematic");
  const copy = body.clone();
  expect(copy.bodyType).toBe("kinematic");
  copy.setVelocity({ linear: new Vector3(5, 0, 0) });
  expect(body.getVelocity().linear.x).toBe(3);
  expect(() => new RigidBody({ type: "static" }).copy(body)).toThrow(
    "immutable",
  );
});

it("expresses complete mass and paired joint frames in the types", () => {
  expectTypeOf<{
    mass: number;
    centerOfMass: Vec3;
  }>().not.toMatchTypeOf<RigidBodyOptions>();
  expectTypeOf<{
    body0: null;
    body1: RigidBody;
    frame0: Matrix4;
  }>().not.toMatchTypeOf<JointOptions>();
  expectTypeOf<{ mass: number }>().toMatchTypeOf<RigidBodyOptions>();
  expectTypeOf<typeof mass>().toMatchTypeOf<RigidBodyOptions>();
});
