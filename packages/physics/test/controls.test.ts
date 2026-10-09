import { expect, expectTypeOf, it } from "vitest";
import { Group, Matrix4 } from "three";
import {
  DistanceJoint,
  MeshCollider,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
  type RigidBodyOptions,
  type JointOptions,
  type Vec3,
  clone,
} from "../src/index.js";
import { initialVelocity, JointBinding, type Motion } from "../src/backend.js";

const mass = {
  mass: 1,
  centerOfMass: [0, 0, 0],
  diagonalInertia: [1, 1, 1],
} satisfies RigidBodyOptions;

it("shares immutable options with clones and copies only between objects that share them", () => {
  const options = { ...mass, centerOfMass: [1, 2, 3] as const };
  const body = new RigidBody(options);
  expectTypeOf<Pick<RigidBody, "options" | "bodyType">>().toEqualTypeOf<
    Readonly<Pick<RigidBody, "options" | "bodyType">>
  >();
  options.mass = 20;
  expect(body.options).toEqual({
    ...mass,
    centerOfMass: [1, 2, 3],
    bodyType: "dynamic",
    colliders: "auto",
    canSleep: true,
  });
  expect(body.clone().options).toBe(body.options);
  const frame0 = new Matrix4().makeTranslation(2, 0, 0);
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
    frame0,
    frame1: new Matrix4(),
    limits: [-1, 2],
  });
  frame0.makeTranslation(10, 0, 0);
  expect(joint.getFrame(0, new Matrix4()).elements[12]).toBe(2);
  expect(joint.options.body1).toBe(body);
  expect(clone(joint).limits).toEqual([-1, 2]);
  expect(() =>
    new RevoluteJoint({ body0: null, body1: body }).copy(joint),
  ).toThrow("immutable");
  const distance = new DistanceJoint({
    body0: null,
    body1: body,
    limits: [1, 3],
  });
  expect(clone(distance).limits).toEqual([1, 3]);
  expect(() =>
    new DistanceJoint({ body0: null, body1: body, limits: [0, 1] }).copy(
      distance,
    ),
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
  "rejects invalid mass properties: %j",
  (options, error) => {
    expect(() => new RigidBody(options)).toThrow(error);
  },
);

it("copies settings into a clone and rejects bodies constructed separately", () => {
  const source = new RigidBody(mass);
  const target = source.clone();
  source.setLinearDamping(1);
  target.copy(source);
  expect(target.linearDamping).toBe(1);
  expect(source.getColliders()).toEqual([]);
  expect(() => new RigidBody(mass).copy(source)).toThrow("immutable");
  const mesh = new MeshCollider({ approximation: "trimesh" });
  expect(mesh.clone().approximation).toBe("trimesh");
  expect(mesh.clone().geometry).toBe(mesh.geometry);
});

it("validates runtime controls before storing and needs a world to simulate", () => {
  const body = new RigidBody().setLinearDamping(2).setGravityScale(-1);
  expect(() => body.setLinearDamping(-1)).toThrow("nonnegative");
  expect(() => body.setGravityScale(Infinity)).toThrow("finite");
  expect(body.linearDamping).toBe(2);
  expect(body.gravityScale).toBe(-1);
  const joint = new RevoluteJoint({
    body0: null,
    body1: body,
  }).setCollideConnected(true);
  joint.setEnabled(false).setEnabled(true);
  expect(() => joint.getState()).toThrow("not under a built world's scene");
  expect(() => body.wake()).toThrow("not under a built world's scene");
});

it("measures prismatic anchor velocity relative to the rotating reference axis", () => {
  const body0 = new RigidBody({ velocity: { angular: [0, 0, 2] } });
  const body1 = new RigidBody({
    velocity: { linear: [3, 0, 0], angular: [0, 0, 4] },
  });
  // Both bodies rotate about their origins, which sit at the world origin.
  const motion: Motion = {
    angular: (body) => initialVelocity(body).angular,
    velocityAt: (body, point) => {
      const { linear, angular } = initialVelocity(body);
      return linear.add(angular.cross(point));
    },
  };
  const joint = new PrismaticJoint({
    body0,
    body1,
    axis: "X",
    frame0: new Matrix4().makeTranslation(0, 1, 0),
    frame1: new Matrix4().makeTranslation(0, 3, 0),
  });
  // Anchor velocities are -2 and 3-12 along X; axis rotation contributes 2*2.
  const { translation, linearVelocity } = new JointBinding(joint).read(motion);
  expect([translation.x, linearVelocity.x]).toEqual([0, -3]);
});

it("rejects standalone joint cloning, which would keep the original bodies", () => {
  const body = new RigidBody();
  const hinge = new RevoluteJoint({ body0: null, body1: body });
  expect(() => hinge.clone()).toThrow("clone(root)");
  expect(() => new Group().add(body, hinge).clone()).toThrow("clone(root)");
});

it("copies immutable body type into clones", () => {
  const options: { bodyType: "dynamic" | "static" } = { bodyType: "dynamic" };
  const body = new RigidBody(options);
  options.bodyType = "static";
  expect(body.bodyType).toBe("dynamic");
  expect(body.clone().bodyType).toBe("dynamic");
  expect(() => new RigidBody({ bodyType: "static" }).copy(body)).toThrow(
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

it("rejects unknown rigid body options and copies onto itself as a no-op", () => {
  // Untyped callers, such as ones written against the old `type` option.
  expect(() => Reflect.construct(RigidBody, [{ type: "static" }])).toThrow(
    "Unknown rigid body option: type",
  );
  const body = new RigidBody().add(new Group());
  expect(body.copy(body)).toBe(body);
  expect(body.children).toHaveLength(1);
});
