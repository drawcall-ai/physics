import { expect, it } from "vitest";
import { Group, Vector3 } from "three";
import {
  BoxCollider,
  FixedJoint,
  RigidBody,
  Trigger,
  getDefaultWorld,
} from "@drawcall/physics";
import { createWorld } from "./fixtures.js";

it("cleans every world registration after a removed listener throws", async () => {
  const world = await createWorld();
  const body = new RigidBody().add(new BoxCollider());
  const trigger = new Trigger().add(new BoxCollider());
  const second = new RigidBody({ type: "static" }).add(new BoxCollider());
  new Group().add(body, trigger, second);
  world.update(world.fixedDelta);
  const error = new Error("removed listener failed");
  trigger.addEventListener("removed", () => {
    throw error;
  });

  expect(() => world.dispose()).toThrow(error);
  expect(world.disposed).toBe(true);
  for (const object of [body, trigger, second]) {
    expect(object.disposed).toBe(true);
    expect(object.parent).toBeNull();
  }
  expect(() => getDefaultWorld()).toThrow();
  expect(() => world.dispose()).not.toThrow();
});

it("removes a body and all connected joints after a joint scene listener throws", async () => {
  const world = await createWorld();
  const body = new RigidBody().add(new BoxCollider());
  const first = new FixedJoint({ body0: null, body1: body });
  const second = new FixedJoint({ body0: null, body1: body });
  new Group().add(body, first, second);
  world.update(world.fixedDelta);
  const error = new Error("joint removed listener failed");
  first.addEventListener("removed", () => {
    throw error;
  });

  expect(() => body.dispose()).toThrow(error);
  expect(body.disposed).toBe(true);
  expect(body.parent).toBeNull();
  expect(first.parent).toBeNull();
  expect(second.disposed).toBe(true);
  expect(second.parent).toBeNull();
  expect(() => world.update(world.fixedDelta)).not.toThrow();
  expect(
    world.raycast(new Vector3(-2, 0, 0), new Vector3(1, 0, 0), 4),
  ).toBeNull();
});
