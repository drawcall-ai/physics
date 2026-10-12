import { expect, it, vi } from "vitest";
import { MeshCollider, RigidBody } from "@drawcall/physics";
import { concave, createWorld } from "./fixtures.js";

vi.mock("../../physics/dist/colliders/decomposition/coacd.js", () => ({
  default: async () => ({
    decompose() {
      throw new Error("CoACD aborted");
    },
  }),
}));
// Parts cached on disk would skip CoACD.
vi.mock("../../physics/dist/colliders/decomposition/cache.js", () => ({
  key: () => undefined,
}));

it("fails every update until a mesh whose decomposition failed is removed", async () => {
  const world = await createWorld();
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    concave(),
  );
  collider.name = "frame";
  const body = new RigidBody().add(collider);
  body.name = "cart";
  world.scene.add(body);
  world.update(0);
  const failed = "Convex decomposition failed for cart/frame: CoACD aborted";
  await expect(world.decompose()).rejects.toThrow(failed);
  expect(() => world.update(world.fixedDelta)).toThrow(failed);
  expect(() => world.update(world.fixedDelta)).toThrow(failed);
  body.removeFromParent();
  world.update(world.fixedDelta);
});
