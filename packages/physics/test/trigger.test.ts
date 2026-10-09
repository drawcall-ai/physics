import { expect, expectTypeOf, it, vi } from "vitest";
import { BoxGeometry, Group, Mesh, Vector3 } from "three";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  Trigger,
  clone,
  type RaycastHit,
} from "../src/index.js";
import { resolveCollisionGroups } from "../src/backend.js";

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
  expect(copiedTrigger.parent).toBe(copiedBody);
  expect(copiedBody.collisionGroups).toEqual(body.collisionGroups);
  expect(copiedTrigger.collisionGroups).toEqual(trigger.collisionGroups);
  expect(copiedTrigger.getColliders()[0]).not.toBe(trigger.getColliders()[0]);
  copiedTrigger.dispatchEvent({ type: "enter", body });
  expect(listener).not.toHaveBeenCalled();
});

it("fails visibly for overlap reads outside every world's scene", () => {
  const trigger = new Trigger();
  const body = new RigidBody();
  expect(() => trigger.getOverlappingBodies()).toThrow(
    "outside every world's scene",
  );
  expect(() => trigger.overlaps(body)).toThrow("outside every world's scene");
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
    object: new BoxCollider(),
    distance: 1,
    point: new Vector3(),
    normal: new Vector3(),
  };
  expectTypeOf(hit.trigger).toEqualTypeOf<Trigger>();
});
