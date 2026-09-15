import { afterEach, beforeEach, expect, it } from "vitest";
import { Group, Quaternion, Vector3 } from "three";
import {
  AuthoringWorld,
  BoxCollider,
  PrismaticJoint,
  RigidBody,
} from "@drawcall/physics";
import { strFromU8, unzipSync } from "fflate";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
});
afterEach(() => world.dispose());

it("roundtrips explicit mass properties under transformed and scaled parents", async () => {
  const parent = new Group();
  parent.position.set(2, 3, 4);
  parent.rotation.set(0.2, 0.3, 0.4);
  parent.scale.setScalar(2);
  const axes = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), 0.7);
  const body = new RigidBody({
    world,
    colliders: false,
    mass: 4,
    centerOfMass: [0.2, -0.3, 0.4],
    diagonalInertia: [2, 3, 4],
    principalAxes: [axes.x, axes.y, axes.z, axes.w],
  });
  body.name = "Body";
  body.scale.set(1, 2, 3);
  const collider = new BoxCollider();
  collider.position.x = 3;
  body.add(collider);
  parent.add(body);
  const exporter = new PhysicsUSDExporter();
  const bytes = await exporter.parseAsync(parent);
  const layer = unzipSync(bytes)["model.usda"];
  if (!layer) throw new Error("Missing physics layer");
  expect(strFromU8(layer)).toContain(
    `quatf physics:principalAxes = (${axes.w}, 0, ${axes.y}, 0)`,
  );
  const imported = new PhysicsUSDLoader().parse(bytes);
  try {
    const loaded = imported.getObjectByName("Body");
    if (!(loaded instanceof RigidBody)) throw new Error("Missing body");
    expect(loaded.options.mass).toBe(4);
    expect(loaded.options.centerOfMass?.[0]).toBeCloseTo(0.2);
    expect(loaded.options.centerOfMass?.[1]).toBeCloseTo(-0.3);
    expect(loaded.options.centerOfMass?.[2]).toBeCloseTo(0.4);
    expect(loaded.options.diagonalInertia).toEqual(
      body.options.diagonalInertia,
    );
    expect(loaded.options.principalAxes).toEqual(body.options.principalAxes);
  } finally {
    imported.dispose();
  }
});

it("imports independently authored colliderless mass properties and USD quaternion order", () => {
  const imported = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Xform "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{
 float physics:mass = 2
 point3f physics:centerOfMass = (1, 2, 3)
 float3 physics:diagonalInertia = (3, 4, 5)
 quatf physics:principalAxes = (0.5, 0.5, 0.5, 0.5)
 double3 xformOp:scale = (2, 3, 4)
 uniform token[] xformOpOrder = ["xformOp:scale"]
}`);
  try {
    const body = imported.getObjectByName("Body");
    if (!(body instanceof RigidBody)) throw new Error("Missing body");
    expect(body.getColliders()).toEqual([]);
    expect(body.options.centerOfMass).toEqual([2, 6, 12]);
    expect(body.options.diagonalInertia).toEqual([3, 4, 5]);
    expect(body.options.principalAxes).toEqual([0.5, 0.5, 0.5, 0.5]);
  } finally {
    imported.dispose();
  }
});

it("roundtrips method-authored velocities and constraint settings", async () => {
  const scene = new Group();
  const body = new RigidBody({
    world,
    colliders: false,
    mass: 2,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [2, 2, 2],
  });
  body.name = "Body";
  body.setVelocity({
    linear: new Vector3(1, 2, 3),
    angular: new Vector3(0, Math.PI, 0),
  });
  const joint = new PrismaticJoint({
    body0: null,
    body1: body,
    limits: [-2, 3],
  })
    .setEnabled(false)
    .setCollideConnected(true);
  scene.add(body, joint);
  const imported = new PhysicsUSDLoader().parse(
    await new PhysicsUSDExporter().parseAsync(scene),
  );
  try {
    const loaded = imported.getObjectByName("Body");
    const constraint = imported
      .getObjectsByProperty("isObject3D", true)
      .find((object) => object instanceof PrismaticJoint);
    if (
      !(loaded instanceof RigidBody) ||
      !(constraint instanceof PrismaticJoint)
    )
      throw new Error("Missing body or joint");
    expect(loaded.getColliders()).toEqual([]);
    expect(loaded.getVelocity().linear.toArray()).toEqual([1, 2, 3]);
    expect(loaded.getVelocity().angular.y).toBeCloseTo(Math.PI);
    expect(constraint.limits).toEqual([-2, 3]);
    expect(constraint.enabled).toBe(false);
    expect(constraint.collideConnected).toBe(true);
  } finally {
    imported.dispose();
  }
});

it("treats USD zero inertia and principal-axis sentinels as inferred properties", () => {
  const imported = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Cube "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
)
{
 float physics:mass = 2
 float3 physics:diagonalInertia = (0, 0, 0)
 quatf physics:principalAxes = (0, 0, 0, 0)
}`);
  try {
    const body = imported.getObjectByName("Body");
    if (!(body instanceof RigidBody)) throw new Error("Missing body");
    expect(body.options.diagonalInertia).toBeUndefined();
    expect(body.options.principalAxes).toBeUndefined();
    expect(body.getColliders()).toHaveLength(1);
  } finally {
    imported.dispose();
  }
});
