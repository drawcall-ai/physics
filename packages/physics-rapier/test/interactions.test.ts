import { expect, it, vi } from "vitest";
import { Group, Matrix4, Vector3 } from "three";
import {
  BoxCollider,
  DistanceJoint,
  FixedJoint,
  RigidBody,
  Trigger,
} from "@drawcall/physics";
import { box, createWorld, inertialBody, steps } from "./fixtures.js";

function region(size = 2) {
  return new Trigger().add(new BoxCollider({ size: [size, size, size] }));
}

it("samples initial overlaps once per step, returns snapshots, and dispatches before after-step", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  world.scene.add(goal, body);
  const order: string[] = [];
  goal.addEventListener("enter", (event) => {
    expect(event.target).toBe(goal);
    expect(event.body).toBe(body);
    expect(goal.overlaps(body)).toBe(true);
    order.push("enter");
  });
  world.onAfterStep(() => order.push("after"));
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
  const body = new RigidBody({ bodyType: "kinematic" });
  body.add(new BoxCollider(), new BoxCollider());
  body.position.x = -1.5;
  world.scene.add(goal, body);
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

it("keeps objects moved within the scene simulated and exits occupants that leave it at the next sync", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  const group = new Group().add(goal, body);
  world.scene.add(group);
  const exit = vi.fn();
  goal.addEventListener("exit", ({ body: other }) => {
    expect(other).toBe(body);
    expect(other.parent).toBeNull();
    expect(goal.getOverlappingBodies()).toEqual([]);
    exit();
  });
  world.update(world.fixedDelta);
  // Detached from their group but still under the scene, both stay simulated.
  world.scene.add(body, goal);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  body.removeFromParent();
  expect(goal.getOverlappingBodies()).toEqual([]);
  expect(exit).toHaveBeenCalledTimes(1);
  // Leaving the scene removes the trigger from the simulation.
  world.scene.remove(goal);
  world.update(world.fixedDelta);
  expect(() => goal.getOverlappingBodies()).toThrow(
    "outside every world's scene",
  );
});

it("resets overlap samples silently and establishes fresh enters on the next step", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("static");
  world.scene.add(goal, body);
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
    const body = box(type);
    world.scene.add(goal, region(), body);
    world.update(world.fixedDelta);
    expect(goal.getOverlappingBodies()).toEqual([body]);
  },
);

it("detects initially sleeping occupants and preserves their overlap while asleep", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box();
  world.scene.add(goal, body);
  world.update(0);
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
  world.scene.add(parent, other);
  world.scene.add(new FixedJoint({ body0: parent, body1: other }));
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
    world.scene.add(body);
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
  const body = new RigidBody({ bodyType: "static" }).setCollisionGroups({
    membership: 4,
    filter: 1,
  });
  const shape = new BoxCollider();
  body.add(shape);
  world.scene.add(goal, body);
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
  world.scene.add(goal, body);
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

it("defers removal exits until the current dispatch completes", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  world.scene.add(goal, body);
  const order: string[] = [];
  goal.addEventListener("enter", () => {
    order.push("enter");
    body.removeFromParent();
    expect(goal.getOverlappingBodies()).toEqual([]);
    order.push("removed");
  });
  goal.addEventListener("exit", () => order.push("exit"));
  world.update(world.fixedDelta);
  expect(order).toEqual(["enter", "removed", "exit"]);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("rejects reentrant simulation and propagates listener errors without replay", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  world.scene.add(goal, body);
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

it("filters solid response as well as events and permits live owner-mask changes", async () => {
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

it("does not dispatch teardown exits while disposing the world", async () => {
  const world = await createWorld();
  const goal = region();
  world.scene.add(goal, box("static"));
  const exit = vi.fn();
  goal.addEventListener("exit", exit);
  world.update(world.fixedDelta);
  world.dispose();
  expect(exit).not.toHaveBeenCalled();
  expect(() => goal.getOverlappingBodies()).toThrow(
    "outside every world's scene",
  );
});

it("finishes body removal even when its exit listener throws", async () => {
  const world = await createWorld();
  const goal = region();
  const body = box("kinematic");
  const attached = region();
  body.add(attached);
  const parent = new Group().add(body);
  world.scene.add(goal, parent);
  const fail = () => {
    throw new Error("exit failed");
  };
  goal.addEventListener("exit", fail);
  world.update(world.fixedDelta);
  expect(goal.overlaps(body)).toBe(true);
  body.removeFromParent();
  expect(() => world.update(world.fixedDelta)).toThrow("exit failed");
  expect(goal.getOverlappingBodies()).toEqual([]);
  expect(() => attached.getOverlappingBodies()).toThrow(
    "outside every world's scene",
  );
  expect(() => body.wake()).toThrow("outside every world's scene");
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
  world.scene.add(parent, other);
  world.update(world.fixedDelta);
  expect(goal.getOverlappingBodies()).toEqual([other]);
  expect(goal.overlaps(parent)).toBe(false);
  parent.teleport(new Matrix4().makeTranslation(5, 0, 0));
  world.update(world.fixedDelta);
  expect(goal.getOverlappingBodies()).toEqual([]);
});

it("does not retain invalid native trigger handles after removing their parent", async () => {
  const world = await createWorld();
  const parent = box("kinematic");
  const trigger = new Trigger().add(new BoxCollider());
  parent.add(trigger);
  const target = box("static");
  world.scene.add(parent, target);
  world.update(world.fixedDelta);
  parent.removeFromParent();
  world.update(world.fixedDelta);
  world.scene.add(parent);
  world.update(world.fixedDelta);
  expect(trigger.overlaps(target)).toBe(true);
});

it("keeps a new body's colliders when a trigger leaves its body as that body leaves", async () => {
  const world = await createWorld();
  const old = box("kinematic");
  const trigger = region();
  old.add(trigger);
  const target = box("static");
  target.position.x = 1;
  world.scene.add(old, target);
  world.update(world.fixedDelta);
  world.scene.attach(trigger);
  old.removeFromParent();
  const next = box("kinematic");
  next.position.x = 10;
  world.scene.add(next);
  world.update(world.fixedDelta);
  expect(
    world.raycast(new Vector3(10, 5, 0), new Vector3(0, -1, 0), 10),
  ).toMatchObject({ kind: "body", body: next });
  expect(trigger.getOverlappingBodies()).toEqual([target]);
});

it("rebuilds a trigger's sensors when its body joins again after a failed join", async () => {
  const world = await createWorld();
  const trigger = region();
  const target = box("static");
  world.scene.add(trigger, target);
  world.update(world.fixedDelta);
  const carrier = box("kinematic");
  const joint = new DistanceJoint({
    body0: null,
    body1: carrier,
    frame0: new Matrix4(),
    frame1: new Matrix4(),
    limits: [0.5, 2],
  });
  carrier.attach(trigger);
  world.scene.add(carrier, joint);
  expect(() => world.update(world.fixedDelta)).toThrow("zero minimum distance");
  joint.removeFromParent();
  world.update(world.fixedDelta);
  expect(trigger.getOverlappingBodies()).toEqual([target]);
  trigger.removeFromParent();
  expect(
    world.raycast(new Vector3(0, 5, 0), new Vector3(0, -1, 0), 10, {
      includeTriggers: true,
    }),
  ).toMatchObject({ kind: "body" });
});

it("delivers the whole batch of contact events even when a listener throws", async () => {
  const world = await createWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 60,
  });
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
