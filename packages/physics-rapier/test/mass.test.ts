import { expect, it } from "vitest";
import {
  BoxGeometry,
  Group,
  Matrix4,
  Mesh,
  PlaneGeometry,
  Quaternion,
  Vector3,
} from "three";
import {
  BoxCollider,
  MeshCollider,
  PrismaticJoint,
  RigidBody,
} from "@drawcall/physics";
import { createWorld, inertialBody } from "./fixtures.js";

for (const c of [
  { density: [1, 9], width: 1, offset: 4, mass: 10, center: 3.6 },
  { density: [0, 0], width: 2, offset: 3, mass: 3, center: 2 },
])
  it(`normalizes collider-derived mass without changing its center: ${JSON.stringify(c)}`, async () => {
    const world = await createWorld();
    const body = new RigidBody({ mass: c.mass });
    const a = new BoxCollider().setMaterial({ density: c.density[0] });
    const b = new BoxCollider({ size: [c.width, 1, 1] }).setMaterial({
      density: c.density[1],
    });
    b.position.x = c.offset;
    body.add(a, b);
    world.update(0);
    body.applyImpulse(new Vector3(0, c.mass, 0), new Vector3(c.center, 0, 0));
    expect(body.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(body.getVelocity().angular.length()).toBeLessThan(1e-6);
  });

it("reads inferred COM immediately and honors later assembly transforms", async () => {
  const world = await createWorld();
  const parent = new Group();
  parent.position.set(10, 20, 30);
  parent.rotation.set(0.2, 0.3, 0.4);
  parent.scale.setScalar(2);
  const body = new RigidBody({ mass: 1 }).setVelocity({
    angular: new Vector3(0, 0, 3).applyQuaternion(parent.quaternion),
  });
  const collider = new BoxCollider();
  collider.position.x = 2;
  body.add(collider);
  parent.add(body);
  const options = {
    body0: null,
    body1: body,
    frame0: new Matrix4().compose(
      parent.position,
      parent.quaternion,
      new Vector3(1, 1, 1),
    ),
    frame1: new Matrix4(),
  };
  const joint = new PrismaticJoint(options).setEnabled(false);
  expect(joint.getState().velocity).toBeCloseTo(-12, 4);
  body.scale.setScalar(1.5);
  expect(joint.getState().velocity).toBeCloseTo(-18, 4);
  world.onBeforeStep(() => {
    expect(joint.getState().velocity).toBeCloseTo(-18, 4);
    expect(new PrismaticJoint(options).getState().velocity).toBeCloseTo(-18, 4);
  });
  world.update(world.fixedDelta);
  expect(world.time).toBe(world.fixedDelta);
});

it("keeps explicit COM and inertia authoritative across geometry changes", async () => {
  const world = await createWorld();
  const body = inertialBody({
    colliders: "auto",
    mass: 2,
    centerOfMass: [1, 0, 0],
    diagonalInertia: [2, 2, 2],
  });
  const mesh = new Mesh(new BoxGeometry(1, 1, 1));
  body.add(mesh);
  for (const size of [1, 4]) {
    mesh.geometry = new BoxGeometry(size, size, size);
    body.setVelocity({ linear: new Vector3(), angular: new Vector3() });
    world.update(0.01);
    body.applyImpulse(new Vector3(0, 2, 0), new Vector3());
    expect(body.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(body.getVelocity().angular.z).toBeCloseTo(-1, 5);
  }
});

it("requires dynamic inertia, permits colliderless anchors, and ignores surface volume for explicit mass", async () => {
  const world = await createWorld();
  for (const mass of [undefined, 1]) {
    const body = new RigidBody({ colliders: false, mass });
    expect(() => world.update(0)).toThrow(/mass|inertia/);
    body.dispose();
  }
  new RigidBody({ type: "static", colliders: false });
  new RigidBody({ type: "kinematic", colliders: false });
  inertialBody({ type: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new PlaneGeometry(1, 1),
    ),
  );
  expect(() => world.update(0)).not.toThrow();
  const invalid = new RigidBody().setMaterial({ density: 0 });
  invalid.add(new BoxCollider());
  expect(() => world.update(0)).toThrow("positive mass");
  invalid.setMaterial({ density: 1 });
  world.update(0);
  invalid.setMaterial({ density: 0 });
  expect(() => world.update(0.01)).toThrow("positive mass");
  invalid.applyImpulse(new Vector3(1, 0, 0));
  expect(invalid.getVelocity().linear.x).toBeCloseTo(1, 5);
});

it("rotates the principal inertia axes used for angular response", async () => {
  const world = await createWorld();
  const axes = new Quaternion().setFromAxisAngle(
    new Vector3(0, 0, 1),
    Math.PI / 2,
  );
  const ordinary = inertialBody({ diagonalInertia: [1, 2, 3] });
  const rotated = inertialBody({
    diagonalInertia: [1, 2, 3],
    principalAxes: [axes.x, axes.y, axes.z, axes.w],
  });
  world.update(0);
  for (const body of [ordinary, rotated])
    body.applyImpulse(new Vector3(0, 0, 1), new Vector3(0, 1, 0));
  expect(ordinary.getVelocity().angular.x).toBeCloseTo(1, 5);
  expect(rotated.getVelocity().angular.x).toBeCloseTo(0.5, 5);
  expect(rotated.getVelocity().angular.y).toBeCloseTo(0, 5);
});
