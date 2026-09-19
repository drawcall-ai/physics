import { afterEach, expect, expectTypeOf, it, vi } from "vitest";
import { BoxGeometry, Group, Mesh, Vector3 } from "three";
import {
  registry,
  BoxCollider,
  MeshCollider,
  RigidBody,
  Trigger,
  clone,
  resolveCollisionGroups,
  type RaycastHit,
} from "../src/index.js";

afterEach(() => {
  registry.clear();
  vi.restoreAllMocks();
});

it("owns only explicit shapes and stops body collider collection at its boundary", () => {
  const body = new RigidBody();
  const mesh = new Mesh(new BoxGeometry());
  const trigger = new Trigger();
  const shape = new BoxCollider();
  trigger.add(new Group().add(shape), new Mesh(new BoxGeometry()));
  body.add(mesh, trigger);
  expect(trigger.getColliders()).toEqual([shape]);
  expect(body.getColliders().map((collider) => collider.source)).toEqual([
    mesh,
  ]);
  expect(new Trigger().getColliders()).toEqual([]);
});

it("resolves complete collider overrides without inheriting a body's masks into its trigger", () => {
  const body = new RigidBody().setCollisionGroups({ membership: 1, filter: 2 });
  const trigger = new Trigger();
  const shape = new BoxCollider();
  body.add(trigger);
  trigger.add(shape);
  expect(resolveCollisionGroups(shape, trigger)).toEqual({
    membership: 65535,
    filter: 65535,
  });
  expect(resolveCollisionGroups(shape, body)).toEqual({
    membership: 1,
    filter: 2,
  });
  const groups = { membership: 4, filter: 8 };
  trigger.setCollisionGroups(groups);
  groups.filter = 0;
  expect(resolveCollisionGroups(shape, trigger)).toEqual({
    membership: 4,
    filter: 8,
  });
  shape.setCollisionGroups({ membership: 0, filter: 0 });
  expect(resolveCollisionGroups(shape, trigger)).toEqual({
    membership: 0,
    filter: 0,
  });
  shape.setCollisionGroups(undefined);
  trigger.setCollisionGroups(undefined);
  expect(resolveCollisionGroups(shape, trigger).filter).toBe(65535);
  expect(() => body.setCollisionGroups({ membership: -1, filter: 1 })).toThrow(
    "16-bit",
  );
  expect(() =>
    trigger.setCollisionGroups({ membership: 1, filter: 1.5 }),
  ).toThrow("16-bit");
});

it("rejects nested owners, physical materials, and surface meshes", () => {
  const trigger = new Trigger();
  const nested = new Trigger();
  trigger.add(nested);
  expect(() => trigger.getColliders()).toThrow("cannot contain");
  trigger.remove(nested);
  trigger.add(new RigidBody());
  expect(() => trigger.getColliders()).toThrow("cannot contain");
  trigger.clear();
  trigger.add(new BoxCollider().setMaterial({ density: 0 }));
  expect(() => trigger.getColliders()).toThrow("materials");
  trigger.clear();
  trigger.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new BoxGeometry(),
    ),
  );
  expect(() => trigger.getColliders()).toThrow("Triangle meshes");
});

it("clones trigger ownership and settings without copying listeners", () => {
  const body = new RigidBody().setCollisionGroups({ membership: 1, filter: 2 });
  const trigger = new Trigger().setCollisionGroups({
    membership: 4,
    filter: 8,
  });
  const listener = vi.fn();
  trigger.addEventListener("enter", listener);
  trigger.add(new BoxCollider());
  body.add(trigger);
  const copiedBody = clone(body);
  const copiedTrigger = copiedBody.children[0];
  if (!(copiedTrigger instanceof Trigger))
    throw new Error("Expected a cloned Trigger");
  expect(registry.objects.has(copiedTrigger)).toBe(true);
  expect(copiedTrigger.parent).toBe(copiedBody);
  expect(copiedBody.collisionGroups).toEqual(body.collisionGroups);
  expect(copiedTrigger.collisionGroups).toEqual(trigger.collisionGroups);
  expect(copiedTrigger.getColliders()[0]).not.toBe(trigger.getColliders()[0]);
  copiedTrigger.dispatchEvent({ type: "enter", body });
  expect(listener).not.toHaveBeenCalled();
  expect(registry.objects.size).toBe(4);
});

it("keeps detachment distinct from disposal and cascades attached trigger disposal", () => {
  const body = new RigidBody();
  const trigger = new Trigger();
  body.add(new Group().add(trigger));
  new Group().add(body).remove(body);
  expect(registry.objects.has(body)).toBe(true);
  expect(trigger.disposed).toBe(false);
  body.dispose();
  expect(trigger.disposed).toBe(true);
  expect(registry.objects.size).toBe(0);
  expect(() => trigger.getOverlappingBodies()).toThrow("disposed");
});

it("detaches a Trigger and preserves both unregister and scene listener failures", () => {
  const trigger = new Trigger();
  new Group().add(trigger);
  const unregisterError = new Error("exit callback failed");
  const removedError = new Error("removed callback failed");
  const unregister = registry.unregister.bind(registry);
  vi.spyOn(registry, "unregister").mockImplementation((object) => {
    unregister(object);
    throw unregisterError;
  });
  trigger.addEventListener("removed", () => {
    throw removedError;
  });

  expect(() => trigger.dispose()).toThrow(
    new AggregateError(
      [unregisterError, removedError],
      "Trigger disposal failed",
    ),
  );
  expect(trigger.parent).toBeNull();
  expect(trigger.disposed).toBe(true);
  expect(registry.objects.size).toBe(0);
  expect(() => trigger.dispose()).not.toThrow();
});

it("finishes disposing every attached Trigger and its body after callback failures", () => {
  const body = new RigidBody();
  const first = new Trigger();
  const second = new Trigger();
  body.add(new Group().add(first, second));
  new Group().add(body);
  const firstError = new Error("first exit callback failed");
  const bodyError = new Error("body exit callback failed");
  const unregister = registry.unregister.bind(registry);
  vi.spyOn(registry, "unregister").mockImplementation((object) => {
    unregister(object);
    if (object === first) throw firstError;
    if (object === body) throw bodyError;
  });

  expect(() => body.dispose()).toThrow(
    new AggregateError([firstError, bodyError], "Rigid body disposal failed"),
  );
  for (const object of [first, second, body]) {
    expect(object.parent).toBeNull();
    expect(object.disposed).toBe(true);
  }
  expect(registry.objects.size).toBe(0);
  expect(() => body.dispose()).not.toThrow();
});

it("fails visibly for authoring overlap reads and invalid arguments", () => {
  const trigger = new Trigger();
  const body = new RigidBody();
  expect(() => trigger.getOverlappingBodies()).toThrow("buildWorld");
  expect(() => trigger.overlaps(body)).toThrow("buildWorld");
  expect(() => trigger.overlaps(new RigidBody())).toThrow("buildWorld");
  body.dispose();
  expect(() => trigger.overlaps(body)).toThrow("disposed");
  expect(registry.objects.has(trigger)).toBe(true);
});

it("types trigger and contact payloads while preserving Three.js scene events", () => {
  const trigger = new Trigger();
  const body = new RigidBody();
  trigger.addEventListener("enter", (event) => {
    expectTypeOf(event.body).toEqualTypeOf<RigidBody>();
    expectTypeOf(event.target).toEqualTypeOf<Trigger>();
  });
  body.addEventListener("contactbegin", (event) => {
    expectTypeOf(event.otherBody).toEqualTypeOf<RigidBody>();
    expectTypeOf(event.target).toEqualTypeOf<RigidBody>();
  });
  trigger.addEventListener("added", (event) =>
    expectTypeOf(event.target).toEqualTypeOf<Trigger>(),
  );
  body.addEventListener("removed", (event) =>
    expectTypeOf(event.target).toEqualTypeOf<RigidBody>(),
  );
  const hit: RaycastHit = {
    kind: "trigger",
    trigger,
    collider: new BoxCollider(),
    distance: 1,
    point: new Vector3(),
    normal: new Vector3(),
  };
  expectTypeOf(hit.trigger).toEqualTypeOf<Trigger>();
});
