import { afterEach, expect, test } from "vitest";
import {
  RigidBody,
  BoxCollider,
  GenericJoint,
  JointDrive,
  jointDofs,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { setupWorld, type MujocoWorld } from "../src/index.js";
const worlds: MujocoWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function world() {
  const value = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  worlds.push(value);
  return value;
}
function free(body1: RigidBody, body0: RigidBody | null = null) {
  return new GenericJoint({
    body0,
    body1,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    dofs: {
      transX: "free",
      transY: "free",
      transZ: "free",
      rotX: "free",
      rotY: "free",
      rotZ: "free",
    },
  });
}
test("angular-only velocity updates preserve center-of-mass linear velocity", async () => {
  const value = await world();
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [1, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  body.add(new BoxCollider());
  value.update(0);
  body.setVelocity({ linear: new Vector3(2, 3, 4) });
  body.setVelocity({ angular: new Vector3(0, 0, 1) });
  expect(body.getVelocity().linear.toArray()).toEqual([2, 3, 4]);
  expect(body.getVelocity().angular.toArray()).toEqual([0, 0, 1]);
});
test("velocity updates copy both caller-owned vectors", async () => {
  const value = await world();
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [1, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  body.add(new BoxCollider());
  value.update(0);
  const linear = new Vector3(4, 5, 6);
  const angular = new Vector3(1, 2, 3);
  body.setVelocity({ linear, angular });
  expect(linear.toArray()).toEqual([4, 5, 6]);
  expect(angular.toArray()).toEqual([1, 2, 3]);
  linear.setScalar(0);
  angular.setScalar(0);
  expect(body.getVelocity().linear.toArray()).toEqual([4, 5, 6]);
  expect(body.getVelocity().angular.toArray()).toEqual([1, 2, 3]);
});
test.each(jointDofs)(
  "free %s acceleration drive uses effective inertia",
  async (axis) => {
    const value = await world();
    const body = new RigidBody({ mass: 1 });
    body.add(new BoxCollider());
    body.scale.set(0.1, 0.2, 0.3);
    const joint = free(body);
    joint.setDrive(
      axis,
      new JointDrive({ damping: 1, model: "acceleration" }).setTarget({
        velocity: 1,
      }),
    );
    value.update(value.fixedDelta);
    expect(joint.getState(axis).velocity).toBeCloseTo(0.01 / 1.01, 10);
  },
);
test("a free drive accounts for both moving endpoints", async () => {
  const value = await world();
  const a = new RigidBody({ mass: 1 });
  a.add(new BoxCollider());
  const b = new RigidBody({ mass: 3 });
  b.add(new BoxCollider());
  b.position.x = 2;
  const joint = free(b, a);
  joint.setDrive(
    "transX",
    new JointDrive({ damping: 1, model: "acceleration" }).setTarget({
      velocity: 1,
    }),
  );
  value.update(value.fixedDelta);
  expect(joint.getState("transX").velocity).toBeCloseTo(0.01 / 1.01, 10);
  expect(a.getVelocity().linear.x).toBeCloseTo((-0.75 * 0.01) / 1.01, 10);
  expect(b.getVelocity().linear.x).toBeCloseTo((0.25 * 0.01) / 1.01, 10);
});
test("free spring predicts motion and includes effort in the implicit solve", async () => {
  const value = await world();
  const body = new RigidBody({ mass: 2 });
  body.add(new BoxCollider());
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  const joint = free(body);
  joint.setDrive(
    "transX",
    new JointDrive({ stiffness: 100, damping: 20 }).setTarget({
      position: 1,
      velocity: 3,
      effort: 7,
    }),
  );
  const force =
    (100 * (1 - 0.01 * 2) + 20 * (3 - 2) + 7) /
    (1 + (20 * 0.01 + 100 * 0.01 ** 2) / 2);
  value.update(value.fixedDelta);
  expect(body.getVelocity().linear.x).toBeCloseTo(2 + (0.01 * force) / 2, 10);
});
test("frame-relative translation applies its reaction at the driven anchor", async () => {
  const value = await world();
  const a = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  a.add(new BoxCollider());
  const b = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  b.add(new BoxCollider());
  b.position.x = 2;
  free(b, a).setDrive("transY", new JointDrive({}).setTarget({ effort: 1 }));
  value.update(value.fixedDelta);
  expect(a.getVelocity().angular.z).toBeCloseTo(-0.02, 8);
  expect(b.getVelocity().angular.z).toBeCloseTo(0, 8);
  expect(a.getVelocity().linear.y).toBeCloseTo(-0.01, 8);
  expect(b.getVelocity().linear.y).toBeCloseTo(0.01, 8);
});

test("free acceleration drive between static endpoints has no response", async () => {
  const value = await world();
  const body = new RigidBody({ type: "static" });
  body.add(new BoxCollider());
  free(body).setDrive(
    "transX",
    new JointDrive({ damping: 1, model: "acceleration" }).setTarget({
      velocity: 1,
    }),
  );
  expect(() => value.update(value.fixedDelta)).not.toThrow();
  expect(body.position.x).toBe(0);
});
