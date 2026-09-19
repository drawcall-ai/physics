import { afterEach, expect, it } from "vitest";
import { Group } from "three";
import { registry, RigidBody } from "@drawcall/physics";
import { PhysicsUSDScene } from "../src/index.js";

afterEach(() => registry.clear());

it("releases attached and detached owned bodies even when removal listeners fail", () => {
  const scene = new PhysicsUSDScene();
  const attached = new RigidBody(),
    detached = new RigidBody();
  scene.add(attached);
  scene.own(detached);
  new Group().add(detached);
  new Group().add(scene);
  const first = new Error("body removal failed");
  const second = new Error("scene removal failed");
  attached.addEventListener("removed", () => {
    throw first;
  });
  scene.addEventListener("removed", () => {
    throw second;
  });
  let failure: unknown;
  try {
    scene.dispose();
  } catch (error) {
    failure = error;
  }
  expect(failure).toBeInstanceOf(AggregateError);
  if (!(failure instanceof AggregateError))
    throw new Error("Expected disposal errors");
  expect(failure.errors).toEqual([first, second]);
  expect(registry.objects.size).toBe(0);
  expect(attached.disposed).toBe(true);
  expect(detached.disposed).toBe(true);
  expect(scene.parent).toBeNull();
  expect(() => scene.dispose()).not.toThrow();
});

it("preserves clone failure while releasing every partially cloned body", () => {
  const removal = new Error("clone removal failed");
  const cloning = new Error("child clone failed");
  class Body extends RigidBody {
    constructor() {
      super();
      this.addEventListener("removed", () => {
        throw removal;
      });
    }
  }
  class Invalid extends Group {
    override clone(): this {
      throw cloning;
    }
  }
  const body = new Body();
  const scene = new PhysicsUSDScene().add(body, new Invalid());
  let failure: unknown;
  try {
    scene.clone();
  } catch (error) {
    failure = error;
  }
  expect(failure).toBeInstanceOf(AggregateError);
  if (!(failure instanceof AggregateError))
    throw new Error("Expected rollback errors");
  expect(failure.errors).toEqual([cloning, removal]);
  expect([...registry.objects]).toEqual([body]);
  expect(() => scene.dispose()).toThrow(removal);
});
