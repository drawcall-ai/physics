import { expect, it } from "vitest";
import { BoxGeometry, Matrix4, Mesh, Vector3 } from "three";
import { BoxCollider, RigidBody } from "@drawcall/physics";
import { createWorld } from "./fixtures.js";

it("queries prepared surfaces, exits, source identity and multiple exclusions after motion/teleport", async () => {
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
  expect(world.raycast(origin, direction, 20)).toBeNull();
  world.update(0);
  expect(world.raycast(new Vector3(), direction, 20)?.distance).toBeCloseTo(1);
  expect(
    world.raycast(origin, new Vector3(Number.MIN_VALUE, 0, 0), 20)?.distance,
  ).toBeCloseTo(2);
  world.update(0.01);
  const hit = world.raycast(origin, direction, 20);
  expect(hit?.body).toBe(first);
  expect(hit?.collider).toBe(first.children[0]);
  expect(hit?.distance).toBeCloseTo(2.01, 5);
  expect(hit?.point.x).toBeCloseTo(-0.99, 5);
  expect(hit?.normal.toArray()).toEqual([-1, 0, 0]);
  expect(
    world.raycast(origin, direction, 20, { excludeBodies: [first, second] })
      ?.body,
  ).toBe(third);
  first.teleport(new Matrix4().makeTranslation(10, 0, 0));
  expect(world.raycast(origin, direction, 20)?.body).toBe(second);
  second.clear().add(new BoxCollider({ size: [4, 1, 1] }));
  world.update(0.01);
  expect(world.raycast(origin, direction, 20)?.collider).toBe(
    second.children[0],
  );
});

it("filters sensors/groups and rejects invalid ray inputs", async () => {
  const world = await createWorld();
  const sensor = new RigidBody({ type: "static" });
  sensor.add(
    new BoxCollider()
      .setSensor(true)
      .setCollisionGroups({ membership: 2, filter: 4 }),
  );
  world.update(0);
  const origin = new Vector3(-3, 0, 0),
    direction = new Vector3(1, 0, 0);
  expect(world.raycast(origin, direction, 10)).toBeNull();
  expect(
    world.raycast(origin, direction, 10, {
      includeSensors: true,
      collisionGroups: { membership: 4, filter: 2 },
    })?.body,
  ).toBe(sensor);
  expect(
    world.raycast(origin, direction, 10, {
      includeSensors: true,
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
    world.raycast(origin, direction, 0, { includeSensors: true }),
  ).toBeNull();
});
