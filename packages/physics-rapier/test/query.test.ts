import { expect, it } from "vitest";
import { createWorld } from "./fixtures.js";
import { BoxGeometry, Matrix4, Mesh, Vector3 } from "three";
import { BoxCollider, RigidBody, Trigger } from "@drawcall/physics";

it("queries authored and simulated surfaces, exits, source identity and multiple exclusions after motion/teleport", async () => {
  const world = await createWorld();
  const bodies = [0, 3, 6].map((x) => {
    const body = new RigidBody({ mass: 1 }).setVelocity({
      linear: new Vector3(1, 0, 0),
    });
    body.position.x = x;
    body.add(new Mesh(new BoxGeometry(2, 2, 2)));
    return body;
  });
  const [first, second, third] = bodies;
  if (!first || !second || !third) throw new Error("Missing test bodies");
  const origin = new Vector3(-3, 0, 0),
    direction = new Vector3(5, 0, 0);
  expect(world.raycast(origin, direction, 20)).toMatchObject({
    kind: "body",
    body: first,
  });
  expect(world.time).toBe(0);
  expect(world.raycast(new Vector3(), direction, 20)?.distance).toBeCloseTo(1);
  expect(
    world.raycast(origin, new Vector3(Number.MIN_VALUE, 0, 0), 20)?.distance,
  ).toBeCloseTo(2);
  world.update(0.01);
  const hit = world.raycast(origin, direction, 20);
  expect(hit).toMatchObject({ kind: "body", body: first });
  expect(hit?.collider).toBe(first.children[0]);
  expect(hit?.distance).toBeCloseTo(2.01, 5);
  expect(hit?.point.x).toBeCloseTo(-0.99, 5);
  expect(hit?.normal.toArray()).toEqual([-1, 0, 0]);
  expect(
    world.raycast(origin, direction, 20, { excludeBodies: [first, second] }),
  ).toMatchObject({ kind: "body", body: third });
  first.teleport(new Matrix4().makeTranslation(10, 0, 0));
  expect(world.raycast(origin, direction, 20)).toMatchObject({
    kind: "body",
    body: second,
  });
  second.clear().add(new BoxCollider({ size: [4, 1, 1] }));
  world.update(0.01);
  expect(world.raycast(origin, direction, 20)?.collider).toBe(
    second.children[0],
  );
});

it("filters triggers/groups and rejects invalid ray inputs", async () => {
  const world = await createWorld();
  const trigger = new Trigger();
  trigger.add(
    new BoxCollider().setCollisionGroups({ membership: 2, filter: 4 }),
  );
  world.update(0);
  const origin = new Vector3(-3, 0, 0),
    direction = new Vector3(1, 0, 0);
  expect(world.raycast(origin, direction, 10)).toBeNull();
  expect(
    world.raycast(origin, direction, 10, {
      includeTriggers: true,
      collisionGroups: { membership: 4, filter: 2 },
    }),
  ).toMatchObject({ kind: "trigger", trigger });
  expect(
    world.raycast(origin, direction, 10, {
      includeTriggers: true,
      collisionGroups: { membership: 4, filter: 1 },
    }),
  ).toBeNull();
  for (const invalid of [
    new Vector3(),
    new Vector3(NaN, 0, 0),
    new Vector3(Infinity, 0, 0),
  ])
    expect(() => world.raycast(origin, invalid, 5)).toThrow();
  for (const distance of [-1, NaN, Infinity])
    expect(() => world.raycast(origin, direction, distance)).toThrow();
  expect(() => world.raycast(new Vector3(NaN, 0, 0), direction, 5)).toThrow();
  expect(
    world.raycast(origin, direction, 0, { includeTriggers: true }),
  ).toBeNull();
});

it("queries unprepared and attached triggers without capturing scale or aliasing their body", async () => {
  const world = await createWorld();
  const body = new RigidBody({ type: "kinematic" }).add(new BoxCollider());
  const trigger = new Trigger().setCollisionGroups({
    membership: 2,
    filter: 4,
  });
  const collider = new BoxCollider();
  trigger.position.x = 3;
  trigger.add(collider);
  body.add(trigger);
  const origin = new Vector3(1, 0, 0),
    direction = new Vector3(1, 0, 0);
  const options = { includeTriggers: true, excludeBodies: [body] };
  const hit = world.raycast(origin, direction, 10, options);
  expect(hit?.kind).toBe("trigger");
  if (hit?.kind !== "trigger") throw new Error("Expected trigger hit");
  expect(hit.trigger).toBe(trigger);
  expect(hit.collider).toBe(collider);
  expect(hit.distance).toBeCloseTo(1.5);
  expect(trigger.getOverlappingBodies()).toEqual([]);
  collider.scale.setScalar(2);
  world.update(world.fixedDelta);
  expect(world.raycast(origin, direction, 10, options)?.distance).toBeCloseTo(
    1,
  );
  body.teleport(new Matrix4().makeTranslation(1, 0, 0));
  expect(world.raycast(origin, direction, 10, options)?.distance).toBeCloseTo(
    2,
  );
  expect(
    world.raycast(origin, direction, 10, {
      ...options,
      collisionGroups: { membership: 1, filter: 2 },
    }),
  ).toBeNull();
});

it("excludes disposed owners from raycasts inside event dispatch", async () => {
  const world = await createWorld();
  const body = new RigidBody({ type: "kinematic" }).add(new BoxCollider());
  const trigger = new Trigger().add(new BoxCollider());
  trigger.addEventListener("enter", () => {
    body.dispose();
    trigger.dispose();
    expect(
      world.raycast(new Vector3(-2, 0, 0), new Vector3(1, 0, 0), 5, {
        includeTriggers: true,
      }),
    ).toBeNull();
  });
  world.update(world.fixedDelta);
});
