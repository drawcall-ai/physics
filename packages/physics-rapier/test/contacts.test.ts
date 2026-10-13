import { expect, test, vi } from "vitest";
import { BoxCollider, MeshCollider, RigidBody } from "@drawcall/physics";
import { BoxGeometry, Matrix4 } from "three";
import { box, createWorld, earth, steps } from "./fixtures.js";

/** A 2 kg, 0.2 m cube resting on a static floor whose top face is y = 0, both colliders rotated. */
async function resting() {
  const world = await createWorld(earth);
  const floor = new RigidBody({ bodyType: "static" });
  floor.add(new BoxCollider({ size: [4, 1, 4] }));
  floor.position.y = -0.5;
  floor.rotation.z = Math.PI;
  const cube = new RigidBody({ mass: 2 });
  cube.add(new BoxCollider({ size: [0.2, 0.2, 0.2] }));
  cube.position.y = 0.1;
  cube.rotation.x = Math.PI / 2;
  world.scene.add(floor, cube);
  return { world, floor, cube };
}

test("contacts carry the normal force the last step solved, pushing each body away from the other", async () => {
  const { world, floor, cube } = await resting();
  expect(cube.getContacts()).toEqual([]);
  steps(world, 100);
  const contacts = cube.getContacts();
  expect(contacts.length).toBeGreaterThan(0);
  const total = contacts.reduce((sum, { force }) => sum + force, 0);
  expect(total).toBeCloseTo(2 * 9.81, 1);
  for (const contact of contacts) {
    expect(contact.otherBody).toBe(floor);
    expect(contact.normal.y).toBeCloseTo(1);
    expect(Math.abs(contact.point.y)).toBeLessThan(0.01);
  }
  for (const { normal } of contacts) normal.negate();
  expect(cube.getContacts().map(({ normal }) => normal.y)).toEqual(
    contacts.map(() => expect.closeTo(1)),
  );
  const reactions = world.getContacts(floor);
  expect(reactions).toHaveLength(contacts.length);
  for (const contact of reactions) {
    expect(contact.otherBody).toBe(cube);
    expect(contact.normal.y).toBeCloseTo(-1);
  }
  cube.removeFromParent();
  expect(floor.getContacts()).toEqual([]);
});

test("contacts clear on reset and agree with contact events", async () => {
  const { world, floor, cube } = await resting();
  cube.position.y = 0.5;
  const touching = (other: RigidBody) =>
    cube.getContacts().some(({ otherBody }) => otherBody === other);
  const begins: boolean[] = [],
    ends: boolean[] = [];
  cube.addEventListener("contactbegin", ({ otherBody }) =>
    begins.push(touching(otherBody)),
  );
  cube.addEventListener("contactend", ({ otherBody }) =>
    ends.push(!touching(otherBody)),
  );
  steps(world, 100);
  cube.teleport(new Matrix4().makeTranslation(0, 0.5, 0));
  steps(world, 1);
  expect(ends.length).toBeGreaterThan(0);
  steps(world, 100);
  expect(begins.length).toBeGreaterThan(1);
  expect([...begins, ...ends].every(Boolean)).toBe(true);
  expect(touching(floor)).toBe(true);
  const end = vi.fn();
  cube.addEventListener("contactend", end);
  world.reset();
  expect(end).not.toHaveBeenCalled();
  expect(cube.getContacts()).toEqual([]);
  expect(floor.getContacts()).toEqual([]);
});

test("aggregates compound solid contacts and notifies both bodies", async () => {
  const world = await createWorld();
  const floor = new RigidBody({ bodyType: "static" });
  floor.add(new BoxCollider(), new BoxCollider());
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider(), new BoxCollider());
  body.position.y = 0.9;
  world.scene.add(floor, body);
  const bodyBegin = vi.fn(),
    floorBegin = vi.fn(),
    bodyEnd = vi.fn(),
    floorEnd = vi.fn();
  body.addEventListener("contactbegin", ({ otherBody }) =>
    bodyBegin(otherBody),
  );
  floor.addEventListener("contactbegin", ({ otherBody }) =>
    floorBegin(otherBody),
  );
  body.addEventListener("contactend", ({ otherBody }) => bodyEnd(otherBody));
  floor.addEventListener("contactend", ({ otherBody }) => floorEnd(otherBody));
  world.update(world.fixedDelta);
  expect(bodyBegin).toHaveBeenCalledExactlyOnceWith(floor);
  expect(floorBegin).toHaveBeenCalledExactlyOnceWith(body);
  body.teleport(new Matrix4().makeTranslation(0, 10, 0));
  world.update(world.fixedDelta);
  expect(bodyEnd).toHaveBeenCalledExactlyOnceWith(floor);
  expect(floorEnd).toHaveBeenCalledExactlyOnceWith(body);
});

test("keeps resting solid contacts through sleep without duplicate transitions", async () => {
  const world = await createWorld({ gravity: [0, -9.81, 0] });
  const floor = box("static");
  const body = box();
  body.position.y = 1;
  world.scene.add(floor, body);
  const begin = vi.fn(),
    end = vi.fn();
  body.addEventListener("contactbegin", begin);
  body.addEventListener("contactend", end);
  steps(world, 120);
  body.sleep();
  steps(world, 10);
  expect(begin).toHaveBeenCalledTimes(1);
  expect(end).not.toHaveBeenCalled();
  expect(body.position.y).toBeCloseTo(1, 1);
  floor.removeFromParent();
  world.update(0);
  expect(end).toHaveBeenCalledTimes(1);
});

test("filters solid response as well as events and permits live owner-mask changes", async () => {
  const world = await createWorld();
  const floor = box("static").setCollisionGroups({ membership: 1, filter: 2 });
  const body = box().setCollisionGroups({ membership: 2, filter: 0 });
  body.position.y = 0.9;
  world.scene.add(floor, body);
  const begin = vi.fn();
  body.addEventListener("contactbegin", begin);
  steps(world, 3);
  expect(body.position.y).toBeCloseTo(0.9, 5);
  expect(begin).not.toHaveBeenCalled();
  body.setCollisionGroups({ membership: 2, filter: 1 });
  steps(world, 3);
  expect(begin).toHaveBeenCalledTimes(1);
  expect(body.position.y).toBeGreaterThan(0.9);
});

test("delivers the whole batch of contact events even when a listener throws", async () => {
  const world = await createWorld(earth);
  const floor = box("static");
  const body = box();
  body.position.y = 1;
  world.scene.add(floor, body);
  const floorBegin = vi.fn();
  body.addEventListener("contactbegin", () => {
    throw new Error("listener failed");
  });
  floor.addEventListener("contactbegin", floorBegin);
  expect(() => world.update(world.fixedDelta)).toThrow("listener failed");
  expect(floorBegin).toHaveBeenCalledOnce();
});

test("reports a body resting on a static triangle mesh floor as on a box floor, without the forces Rapier does not expose", async () => {
  /** The steps at which a box dropped on the floor begins and ends contact, and the floor's final contact forces. */
  async function drop(collider: BoxCollider | MeshCollider) {
    const world = await createWorld(earth);
    const floor = new RigidBody({ bodyType: "static" }).add(collider);
    const body = box();
    body.position.y = 1.5;
    world.scene.add(floor, body);
    const events: string[] = [];
    for (const type of ["contactbegin", "contactend"] as const)
      floor.addEventListener(type, () => events.push(`${type}@${step}`));
    let step = 0;
    for (; step < 120; step++) world.update(world.fixedDelta);
    return { events, forces: floor.getContacts().map(({ force }) => force) };
  }
  const boxFloor = await drop(new BoxCollider({ size: [10, 1, 10] }));
  expect(boxFloor.events).toEqual(["contactbegin@19"]);
  const meshFloor = await drop(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new BoxGeometry(10, 1, 10),
    ),
  );
  expect(meshFloor.events).toEqual(boxFloor.events);
  expect(meshFloor.forces.length).toBeGreaterThan(0);
  expect(meshFloor.forces.every((force) => force === 0)).toBe(true);
});
