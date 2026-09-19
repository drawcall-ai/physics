import { expect, it, vi } from "vitest";
import { Group, Matrix4, Vector3 } from "three";
import { BoxCollider, FixedJoint, RigidBody, Trigger } from "@drawcall/physics";
import { box, createWorld, inertialBody, steps } from "./fixtures.js";

function region(size = 2) {
  return new Trigger().add(new BoxCollider({ size: [size, size, size] }));
}

it("samples initial overlaps once per step, returns snapshots, and dispatches before after-step", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  const order: string[] = [];
  goal.addEventListener("enter", (event) => {
    expect(event.target).toBe(goal);
    expect(event.body).toBe(body);
    expect(goal.overlaps(body)).toBe(true);
    order.push("enter");
  });
  world.onAfterStep(() => order.push("after"));
  expect(goal.overlaps(body)).toBe(false);
  world.update(0);
  expect(goal.getOverlappingBodies()).toEqual([]);
  world.update(world.fixedDelta);
  expect(order).toEqual(["enter", "after"]);
  const snapshot = goal.getOverlappingBodies();
  snapshot.length = 0;
  expect(goal.getOverlappingBodies()).toEqual([body]);
  body.teleport(new Matrix4().makeTranslation(10, 0, 0));
  expect(goal.overlaps(body)).toBe(true);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(false);
});

it("aggregates compound body and trigger shapes without shape-handoff noise", async () => {
  const world = await createWorld();
  const goal = new Trigger();
  const left = new BoxCollider({ size: [2, 2, 2] });
  const right = left.clone();
  left.position.x = -1;
  right.position.x = 1;
  goal.add(left, right);
  const body = new RigidBody({ type: "kinematic" });
  body.add(new BoxCollider(), new BoxCollider());
  body.position.x = -1.5;
  const enter = vi.fn(),
    exit = vi.fn();
  goal.addEventListener("enter", enter);
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  body.teleport(new Matrix4().makeTranslation(1.5, 0, 0));
  world.update(world.fixedDelta);
  expect(enter).toHaveBeenCalledTimes(1);
  expect(exit).not.toHaveBeenCalled();
  body.teleport(new Matrix4().makeTranslation(5, 0, 0));
  world.update(world.fixedDelta);
  expect(exit).toHaveBeenCalledTimes(1);
});

it("does not equate scene detachment with disposal and clears disposed occupants immediately", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  const scene = new Group().add(goal, body);
  const exit = vi.fn();
  goal.addEventListener("exit", ({ body: other }) => {
    expect(other.disposed).toBe(true);
    expect(goal.getOverlappingBodies()).toEqual([]);
    exit();
  });
  world.update(world.fixedDelta);
  scene.remove(body, goal);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  body.dispose();
  expect(exit).toHaveBeenCalledTimes(1);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("resets overlap samples silently and establishes fresh enters on the next step", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  const enter = vi.fn(),
    exit = vi.fn();
  goal.addEventListener("enter", enter);
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  world.reset();
  expect(goal.overlaps(body)).toBe(false);
  expect(exit).not.toHaveBeenCalled();
  world.update(world.fixedDelta);
  expect(enter).toHaveBeenCalledTimes(2);
});

it.each(["static", "kinematic", "dynamic"] as const)(
  "detects %s occupants and never detects another trigger",
  async (type) => {
    const world = await createWorld();
    const goal = region();
    region();
    const body = box(type);
    world.update(world.fixedDelta);
    expect(goal.getOverlappingBodies()).toEqual([body]);
  },
);

it("detects initially sleeping occupants and preserves their overlap while asleep", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box();
  body.sleep();
  const enter = vi.fn(),
    exit = vi.fn();
  goal.addEventListener("enter", enter);
  goal.addEventListener("exit", exit);
  steps(world, 10);
  expect(goal.overlaps(body)).toBe(true);
  expect(enter).toHaveBeenCalledTimes(1);
  expect(exit).not.toHaveBeenCalled();
});

it("attaches to body motion, excludes its parent, and detects a joint neighbor", async () => {
  const world = await createWorld();
  const parent = box("kinematic");
  const other = box("kinematic");
  other.position.x = 3;
  const goal = region();
  goal.position.x = 1;
  parent.add(new Group().add(goal));
  new FixedJoint({ body0: parent, body1: other });
  world.update(world.fixedDelta);
  expect(goal.overlaps(parent)).toBe(false);
  expect(goal.overlaps(other)).toBe(false);
  parent.teleport(new Matrix4().makeTranslation(2, 0, 0));
  world.update(world.fixedDelta);
  expect(goal.overlaps(other)).toBe(true);
  expect(goal.overlaps(parent)).toBe(false);
});

it.each(["inferred", "normalized", "explicit"])(
  "does not add mass or inertia to a body with %s mass",
  async (mode) => {
    const world = await createWorld();
    const body =
      mode === "explicit"
        ? inertialBody({ mass: 2, diagonalInertia: [2, 2, 2] })
        : new RigidBody(mode === "normalized" ? { mass: 2 } : {});
    body.add(new BoxCollider().setMaterial({ density: 2 }));
    const goal = region(10);
    goal.position.x = 10;
    body.add(goal);
    world.update(0);
    body.applyImpulse(new Vector3(0, 2, 0), new Vector3());
    expect(body.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(body.getVelocity().angular.length()).toBeLessThan(1e-6);
    body.setVelocity({ linear: new Vector3(), angular: new Vector3() });
    body.applyImpulse(new Vector3(0, 2, 0), new Vector3(1, 0, 0));
    expect(body.getVelocity().angular.z).toBeCloseTo(
      mode === "explicit" ? 1 : 6,
      4,
    );
  },
);

it("applies owner masks, complete collider overrides, and live filter changes", async () => {
  const world = await createWorld();
  const goal = region().setCollisionGroups({ membership: 1, filter: 2 });
  const body = new RigidBody({ type: "static" }).setCollisionGroups({
    membership: 4,
    filter: 1,
  });
  const shape = new BoxCollider();
  body.add(shape);
  const enter = vi.fn(),
    exit = vi.fn();
  goal.addEventListener("enter", enter);
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(false);
  shape.setCollisionGroups({ membership: 2, filter: 1 });
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  body.setCollisionGroups({ membership: 0, filter: 0 });
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  expect(enter).toHaveBeenCalledTimes(1);
  shape.setCollisionGroups(undefined);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(false);
  expect(exit).toHaveBeenCalledTimes(1);
});

it("keeps overlap state through unrelated material and body setting rebuilds", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box();
  const enter = vi.fn(),
    exit = vi.fn();
  goal.addEventListener("enter", enter);
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  body.setMaterial({ density: 2 }).setLinearDamping(0.2);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  expect(enter).toHaveBeenCalledTimes(1);
  expect(exit).not.toHaveBeenCalled();
});

it("aggregates compound solid contacts and notifies both bodies", async () => {
  const world = await createWorld();
  const floor = new RigidBody({ type: "static" });
  floor.add(new BoxCollider(), new BoxCollider());
  const body = new RigidBody({ mass: 1 });
  body.add(new BoxCollider(), new BoxCollider());
  body.position.y = 0.9;
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

it("defers disposal exits until the current dispatch completes", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  const order: string[] = [];
  goal.addEventListener("enter", () => {
    order.push("enter");
    body.dispose();
    order.push("disposed");
  });
  goal.addEventListener("exit", () => order.push("exit"));
  world.update(world.fixedDelta);
  expect(order).toEqual(["enter", "disposed", "exit"]);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("rejects reentrant simulation and propagates listener errors without replay", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  const after = vi.fn();
  world.onAfterStep(after);
  const listener = () => {
    expect(() => world.update(world.fixedDelta)).toThrow();
    expect(() => world.reset()).toThrow();
    throw new Error("listener failed");
  };
  goal.addEventListener("enter", listener);
  expect(() => world.update(world.fixedDelta)).toThrow("listener failed");
  expect(goal.overlaps(body)).toBe(true);
  expect(after).not.toHaveBeenCalled();
  goal.removeEventListener("enter", listener);
  const late = vi.fn();
  goal.addEventListener("enter", late);
  world.update(world.fixedDelta);
  expect(late).not.toHaveBeenCalled();
  expect(after).toHaveBeenCalledTimes(1);
});

it("keeps resting solid contacts through sleep without duplicate transitions", async () => {
  const world = await createWorld({ gravity: [0, -9.81, 0] });
  const floor = box("static");
  const body = box();
  body.position.y = 1;
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
  floor.dispose();
  expect(end).toHaveBeenCalledTimes(1);
});

it("filters solid response as well as events and permits live owner-mask changes", async () => {
  const world = await createWorld();
  box("static").setCollisionGroups({ membership: 1, filter: 2 });
  const body = box().setCollisionGroups({ membership: 2, filter: 0 });
  body.position.y = 0.9;
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

it("does not dispatch teardown exits while disposing the world", async () => {
  const world = await createWorld();
  const goal = region();
  box("static");
  const exit = vi.fn();
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  world.dispose();
  expect(exit).not.toHaveBeenCalled();
  expect(() => goal.getOverlappingBodies()).toThrow("disposed");
});

it("finishes body disposal even when its exit listener throws", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  const attached = region();
  body.add(attached);
  const parent = new Group().add(body);
  const fail = () => {
    throw new Error("exit failed");
  };
  goal.addEventListener("exit", fail);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  expect(() => body.dispose()).toThrow("exit failed");
  expect(body.disposed).toBe(true);
  expect(attached.disposed).toBe(true);
  expect(body.parent).toBe(null);
  expect(parent.children).toEqual([]);
  expect(goal.getOverlappingBodies()).toEqual([]);
  goal.removeEventListener("exit", fail);
  world.update(world.fixedDelta);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("detects separate static bodies from a static attachment while excluding its parent", async () => {
  const world = await createWorld();
  const parent = box("static");
  const other = box("static");
  const goal = region();
  parent.add(goal);
  world.update(world.fixedDelta);
  expect(goal.getOverlappingBodies()).toEqual([other]);
  expect(goal.overlaps(parent)).toBe(false);
  parent.teleport(new Matrix4().makeTranslation(5, 0, 0));
  world.update(world.fixedDelta);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("does not retain invalid native trigger handles after unregistering their parent", async () => {
  const world = await createWorld();
  const parent = box("kinematic");
  const trigger = new Trigger().add(new BoxCollider());
  parent.add(trigger);
  const target = box("static");
  world.update(world.fixedDelta);
  world.unregister(parent);
  world.register(parent);
  world.update(world.fixedDelta);
  expect(trigger.overlaps(target)).toBe(true);
});

it("delivers the whole batch of contact events even when a listener throws", async () => {
  const world = await createWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 60,
  });
  const floor = box("static");
  const body = box();
  body.position.y = 1;
  const floorBegin = vi.fn();
  body.addEventListener("contactbegin", () => {
    throw new Error("listener failed");
  });
  floor.addEventListener("contactbegin", floorBegin);
  expect(() => world.update(world.fixedDelta)).toThrow("listener failed");
  expect(floorBegin).toHaveBeenCalledOnce();
});
