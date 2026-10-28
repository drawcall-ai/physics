import { expect, test } from "vitest";
import { BoxCollider, RigidBody } from "@drawcall/physics";
import { Vector3 } from "three";
import { createWorld } from "./fixtures.js";

test("torque is world-space, additive, copied, and lasts one fixed step", async () => {
  const world = await createWorld();
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [2, 3, 4],
    colliders: false,
  });
  body.rotation.z = Math.PI / 2;
  world.scene.add(body);
  const torque = new Vector3(0, 4, 0);
  body.applyTorque(torque);
  world.applyTorque(body, new Vector3(0, 2, 0));
  body.applyForce(new Vector3(1, 0, 0));
  torque.setScalar(0);
  world.update(world.fixedDelta);
  expect(
    body.getVelocity().angular.distanceTo(new Vector3(0, 0.03, 0)),
  ).toBeLessThan(1e-6);
  expect(
    body.getVelocity().linear.distanceTo(new Vector3(0.01, 0, 0)),
  ).toBeLessThan(1e-6);
  world.update(world.fixedDelta * 2);
  expect(body.getVelocity().angular.y).toBeCloseTo(0.03, 6);
  expect(body.getVelocity().linear.x).toBeCloseTo(0.01, 6);
  world.reset();
  body.applyTorque(new Vector3(0, 100, 0));
  world.reset();
  world.update(world.fixedDelta);
  expect(body.getVelocity().angular.length()).toBe(0);
});

test.each([false, true])(
  "point velocity uses the %s explicit COM in world space",
  async (explicit) => {
    const world = await createWorld();
    const body = new RigidBody(
      explicit
        ? { mass: 1, centerOfMass: [1, 0, 0], diagonalInertia: [1, 1, 1] }
        : { mass: 1 },
    );
    const collider = new BoxCollider();
    collider.position.x = 1;
    body.add(collider);
    body.position.set(4, 5, 6);
    body.rotation.y = Math.PI / 2;
    world.scene.add(body);
    body.setVelocity({
      linear: new Vector3(1, 2, 3),
      angular: new Vector3(0, 0, 2),
    });
    const point = new Vector3(5, 5, 5);
    const expected = new Vector3(1, 4, 3);
    expect(body.getVelocityAtPoint(point).distanceTo(expected)).toBeLessThan(
      1e-5,
    );
    expect(
      world
        .getVelocityAtPoint(body, new Vector3(4, 5, 5))
        .distanceTo(new Vector3(1, 2, 3)),
    ).toBeLessThan(1e-5);
    expect(point.toArray()).toEqual([5, 5, 5]);
    body.getVelocityAtPoint(point).setScalar(99);
    expect(body.getVelocityAtPoint(point).distanceTo(expected)).toBeLessThan(
      1e-5,
    );
  },
);

test("removed bodies lose queued torque before rejoining", async () => {
  const world = await createWorld();
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
    colliders: false,
  });
  world.scene.add(body);
  body.applyTorque(new Vector3(1, 0, 0));
  body.removeFromParent();
  world.update(0);
  world.scene.add(body);
  world.update(world.fixedDelta);
  expect(body.getVelocity().angular.length()).toBe(0);
});

test("point velocity and torque reject invalid commands", async () => {
  const world = await createWorld();
  const dynamic = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const fixed = new RigidBody({ bodyType: "static" }).add(new BoxCollider());
  const kinematic = new RigidBody({ bodyType: "kinematic" }).add(
    new BoxCollider(),
  );
  kinematic.position.x = 4;
  fixed.position.x = 8;
  world.scene.add(dynamic, fixed, kinematic);
  for (const invalid of [NaN, Infinity, -Infinity]) {
    const vector = new Vector3(invalid, 0, 0);
    expect(() => dynamic.applyTorque(vector)).toThrow("finite");
    expect(() => dynamic.getVelocityAtPoint(vector)).toThrow("finite");
  }
  for (const body of [fixed, kinematic]) {
    expect(() => body.applyTorque(new Vector3())).toThrow("dynamic");
    expect(body.getVelocityAtPoint(new Vector3()).length()).toBe(0);
  }
  dynamic.removeFromParent();
  expect(() => world.applyTorque(dynamic, new Vector3())).toThrow();
  expect(() => world.getVelocityAtPoint(dynamic, new Vector3())).toThrow();
  world.dispose();
  expect(() => world.applyTorque(fixed, new Vector3())).toThrow();
  expect(() => world.getVelocityAtPoint(fixed, new Vector3())).toThrow(
    "disposed",
  );
});
