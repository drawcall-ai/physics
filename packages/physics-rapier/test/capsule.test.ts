import { expect, it } from "vitest";
import { Matrix4, Quaternion, Vector3 } from "three";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  RigidBody,
  Trigger,
} from "@drawcall/physics";
import { createWorld } from "./fixtures.js";

const capsule = { radius: 0.4, height: 1 };

function box(
  size: [number, number, number],
  position: [number, number, number],
) {
  const collider = new BoxCollider({ size });
  const body = new RigidBody({ bodyType: "static" }).add(collider);
  body.position.set(...position);
  return { body, collider };
}

it("sweeps the whole capsule through thin walls and floors without a simulation step", async () => {
  const world = await createWorld();
  const floor = box([20, 1, 20], [0, -0.5, 0]);
  const wall = box([0.1, 6, 20], [2, 2, 0]);
  world.scene.add(floor.body, wall.body);
  const pose = new Matrix4().makeTranslation(0, 2, 0);
  const horizontal = world.castCapsule(capsule, pose, new Vector3(10, 0, 0));
  expect(horizontal?.body).toBe(wall.body);
  expect(horizontal?.object).toBe(wall.collider);
  expect(horizontal?.fraction).toBeCloseTo(0.155, 3);
  expect(horizontal?.point.x).toBeCloseTo(1.95, 3);
  expect(horizontal?.normal.x).toBeCloseTo(-1, 3);
  const vertical = world.castCapsule(capsule, pose, new Vector3(0, -10, 0));
  expect(vertical?.body).toBe(floor.body);
  expect(vertical?.fraction).toBeCloseTo(0.11, 3);
  expect(vertical?.point.y).toBeCloseTo(0, 3);
  expect(vertical?.normal.y).toBeCloseTo(1, 3);
  expect(world.time).toBe(0);
});

it("reports signed contacts on the obstacle, including a predictive gap", async () => {
  const world = await createWorld();
  const floor = box([20, 1, 20], [0, -0.5, 0]);
  world.scene.add(floor.body);
  const [penetration] = world.capsuleContacts(
    capsule,
    new Matrix4().makeTranslation(0, 0.85, 0),
    0,
  );
  expect(penetration?.body).toBe(floor.body);
  expect(penetration?.object).toBe(floor.collider);
  expect(penetration?.distance).toBeCloseTo(-0.05, 4);
  expect(penetration?.normal.y).toBeCloseTo(1, 4);
  expect(penetration?.point.y).toBeCloseTo(0, 4);
  const pose = new Matrix4().makeTranslation(0, 1, 0);
  expect(world.capsuleContacts(capsule, pose, 0.05)).toEqual([]);
  expect(world.capsuleContacts(capsule, pose, 0.15)[0]?.distance).toBeCloseTo(
    0.1,
    3,
  );
});

it.each([
  {
    name: "concentric cylinder",
    collider: () => new CylinderCollider({ radius: 1, height: 3 }),
    x: 0,
  },
  {
    name: "nearly concentric cylinder",
    collider: () => new CylinderCollider({ radius: 1, height: 3 }),
    x: 0.001,
  },
  {
    name: "concentric capsule",
    collider: () => new CapsuleCollider({ radius: 1, height: 1 }),
    x: 0,
  },
])(
  "rejects an unresolved native overlap with a $name",
  async ({ collider, x }) => {
    const world = await createWorld();
    world.scene.add(new RigidBody({ bodyType: "static" }).add(collider()));
    const pose = new Matrix4().makeTranslation(x, 0, 0);
    expect(() => world.capsuleContacts(capsule, pose, 0)).toThrow(
      "Rapier capsule query could not resolve a penetration witness",
    );
    expect(() =>
      world.castCapsule(capsule, pose, new Vector3(1, 0, 0)),
    ).toThrow("Rapier capsule query could not resolve a penetration witness");
  },
);

it("accepts valid zero-distance witnesses on touching capsule surfaces", async () => {
  const world = await createWorld();
  const obstacle = new RigidBody({ bodyType: "static" }).add(
    new CapsuleCollider({ radius: 1, height: 1 }),
  );
  world.scene.add(obstacle);
  const pose = new Matrix4().makeTranslation(1.4, 0, 0);
  const [contact] = world.capsuleContacts(capsule, pose, 0);
  expect(contact?.body).toBe(obstacle);
  expect(contact?.distance).toBe(0);
  expect(world.castCapsule(capsule, pose, new Vector3(1, 0, 0))).toBeNull();
});

it("allows tangent and escaping motion while still finding a wall beyond the floor", async () => {
  const world = await createWorld();
  const floor = box([20, 1, 20], [0, -0.5, 0]);
  const wall = box([0.1, 6, 20], [2, 2, 0]);
  world.scene.add(floor.body, wall.body);
  const pose = new Matrix4().makeTranslation(0, 0.9, 0);
  expect(world.castCapsule(capsule, pose, new Vector3(1, 0, 0))).toBeNull();
  expect(world.castCapsule(capsule, pose, new Vector3(0, 1, 0))).toBeNull();
  expect(world.castCapsule(capsule, pose, new Vector3(3, 0, 0))?.body).toBe(
    wall.body,
  );
  const penetrating = new Matrix4().makeTranslation(0, 0.85, 0);
  expect(
    world.castCapsule(capsule, penetrating, new Vector3(1, 0, 0)),
  ).toBeNull();
  expect(
    world.castCapsule(capsule, penetrating, new Vector3(0, 1, 0)),
  ).toBeNull();
  const blocking = world.castCapsule(
    capsule,
    penetrating,
    new Vector3(0, -1, 0),
  );
  expect(blocking?.fraction).toBe(0);
  expect(blocking?.distance).toBeCloseTo(-0.05, 4);
  expect(world.castCapsule(capsule, pose, new Vector3())).toBeNull();
});

it("honors both collision masks, exclusions and trigger filtering", async () => {
  const world = await createWorld();
  const obstacle = box([1, 4, 4], [2, 0, 0]);
  obstacle.collider.setCollisionGroups({ membership: 2, filter: 4 });
  const trigger = new Trigger().add(new BoxCollider({ size: [1, 4, 4] }));
  trigger.position.x = 1;
  world.scene.add(obstacle.body, trigger);
  const pose = new Matrix4();
  const motion = new Vector3(5, 0, 0);
  const included = { collisionGroups: { membership: 4, filter: 2 } };
  expect(world.castCapsule(capsule, pose, motion, included)?.body).toBe(
    obstacle.body,
  );
  expect(
    world.castCapsule(capsule, pose, motion, {
      excludeBodies: [obstacle.body],
    }),
  ).toBeNull();
  expect(
    world.castCapsule(capsule, pose, motion, {
      collisionGroups: { membership: 1, filter: 2 },
    }),
  ).toBeNull();
  expect(
    world.castCapsule(capsule, pose, motion, {
      collisionGroups: { membership: 4, filter: 1 },
    }),
  ).toBeNull();
  const overlapping = new Matrix4().makeTranslation(2, 0, 0);
  expect(world.capsuleContacts(capsule, overlapping, 0, included)).toHaveLength(
    1,
  );
  expect(
    world.capsuleContacts(capsule, overlapping, 0, {
      excludeBodies: [obstacle.body],
    }),
  ).toEqual([]);
});

it("uses live teleports and rebuilt colliders before the next solver step", async () => {
  const world = await createWorld();
  const obstacle = box([1, 4, 4], [2, 0, 0]);
  world.scene.add(obstacle.body);
  const pose = new Matrix4();
  const motion = new Vector3(10, 0, 0);
  expect(world.castCapsule(capsule, pose, motion)?.fraction).toBeCloseTo(
    0.11,
    3,
  );
  obstacle.body.teleport(new Matrix4().makeTranslation(4, 0, 0));
  expect(world.castCapsule(capsule, pose, motion)?.fraction).toBeCloseTo(
    0.31,
    3,
  );
  const replacement = new BoxCollider({ size: [2, 4, 4] });
  obstacle.body.clear().add(replacement);
  const hit = world.castCapsule(capsule, pose, motion);
  expect(hit?.fraction).toBeCloseTo(0.26, 3);
  expect(hit?.object).toBe(replacement);
  expect(world.time).toBe(0);
  obstacle.body.removeFromParent();
  expect(world.castCapsule(capsule, pose, motion)).toBeNull();
});

it("transforms collider witnesses and normals to world space and respects capsule rotation", async () => {
  const world = await createWorld();
  const obstacle = box([10, 0.2, 10], [3, 0, 0]);
  obstacle.collider.rotation.z = Math.PI / 2;
  world.scene.add(obstacle.body);
  const pose = new Matrix4().makeRotationZ(Math.PI / 2);
  const hit = world.castCapsule(capsule, pose, new Vector3(6, 0, 0));
  expect(hit?.fraction).toBeCloseTo(1 / 3, 3);
  expect(hit?.normal.x).toBeCloseTo(-1, 3);
  expect(hit?.normal.y).toBeCloseTo(0, 3);
  expect(hit?.point.x).toBeCloseTo(2.9, 3);
  const touching = new Matrix4().compose(
    new Vector3(2.1, 0, 0),
    new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2),
    new Vector3(1, 1, 1),
  );
  const [contact] = world.capsuleContacts(capsule, touching, 0);
  expect(contact?.distance).toBeCloseTo(-0.1, 3);
  expect(contact?.normal.x).toBeCloseTo(-1, 3);
  expect(contact?.point.x).toBeCloseTo(2.9, 3);
});

it("moves a crouched capsule through a narrow floor and ceiling gap without false contacts", async () => {
  const world = await createWorld();
  const floor = box([30, 0.2, 30], [0, -0.1, 0]);
  const roof = box([4, 0.3, 3], [2, 1.3, 0]);
  world.scene.add(floor.body, roof.body);
  const crouched = { radius: 0.35, height: 0.416 };
  const pose = new Matrix4().makeTranslation(
    0.1583173,
    0.561948825,
    0.00001455235,
  );
  const motion = new Vector3(0.0366658168, 0.00000560324, 0.00000081839);
  expect(world.capsuleContacts(crouched, pose, 0)).toEqual([]);
  expect(world.castCapsule(crouched, pose, motion)).toBeNull();
  expect(world.castCapsule(crouched, pose, new Vector3(2, 0, 0))).toBeNull();
  const downward = world.castCapsule(crouched, pose, new Vector3(0, -0.01, 0));
  expect(downward?.body).toBe(floor.body);
  expect(downward?.fraction).toBeGreaterThan(0.3);
  expect(downward?.fraction).toBeLessThan(0.5);
});
