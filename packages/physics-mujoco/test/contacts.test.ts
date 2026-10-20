import { expect, test, vi } from "vitest";
import { BoxCollider, RigidBody, type Body } from "@drawcall/physics";
import { Matrix4 } from "three";
import { createWorld, earth, ground, scene, steps } from "./fixtures.js";

/** A 2 kg, 0.2 m box resting on the ground. */
function box(): RigidBody {
  const box = new RigidBody({ mass: 2 });
  box.add(new BoxCollider({ size: [0.2, 0.2, 0.2] }));
  box.position.y = 0.1;
  scene.add(box);
  return box;
}

test("contacts carry the normal force the last step solved, pushing each body away from the other", async () => {
  const floor = ground();
  const resting = box();
  const world = await createWorld(earth);
  expect(resting.getContacts()).toEqual([]);
  steps(world, 100);
  const contacts = resting.getContacts();
  expect(contacts.length).toBeGreaterThan(0);
  const total = contacts.reduce((sum, { force }) => sum + force, 0);
  expect(total).toBeCloseTo(2 * 9.81, 1);
  for (const contact of contacts) {
    expect(contact.otherBody).toBe(floor);
    expect(contact.normal.y).toBeCloseTo(1);
    expect(Math.abs(contact.point.y)).toBeLessThan(0.01);
  }
  const reactions = world.getContacts(floor);
  expect(reactions).toHaveLength(contacts.length);
  for (const contact of reactions) {
    expect(contact.otherBody).toBe(resting);
    expect(contact.normal.y).toBeCloseTo(-1);
  }
  resting.removeFromParent();
  expect(floor.getContacts()).toEqual([]);
});

test("contacts clear on reset and agree with contact events", async () => {
  const floor = ground();
  const falling = box();
  falling.position.y = 0.5;
  const world = await createWorld(earth);
  const touching = (other: Body) =>
    falling.getContacts().some(({ otherBody }) => otherBody === other);
  const begins: boolean[] = [],
    ends: boolean[] = [];
  falling.addEventListener("contactbegin", ({ otherBody }) =>
    begins.push(touching(otherBody)),
  );
  falling.addEventListener("contactend", ({ otherBody }) =>
    ends.push(!touching(otherBody)),
  );
  steps(world, 100);
  falling.teleport(new Matrix4().makeTranslation(0, 0.5, 0));
  steps(world, 1);
  expect(ends.length).toBeGreaterThan(0);
  steps(world, 100);
  expect(begins.length).toBeGreaterThan(1);
  expect([...begins, ...ends].every(Boolean)).toBe(true);
  expect(touching(floor)).toBe(true);
  const end = vi.fn();
  falling.addEventListener("contactend", end);
  world.reset();
  expect(end).not.toHaveBeenCalled();
  expect(falling.getContacts()).toEqual([]);
  expect(floor.getContacts()).toEqual([]);
});
