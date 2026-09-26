import { afterEach, expect, it, vi } from "vitest";
import { BoxGeometry } from "three";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  prepareConvexParts,
  registry,
} from "../src/index.js";

const load = vi.fn();
vi.mock("../src/coacd.js", () => ({ default: load }));

afterEach(() => registry.clear());

it("loads CoACD only when some mesh needs decomposing", async () => {
  const box = new RigidBody().add(new BoxCollider());
  const scenery = new RigidBody({ type: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new BoxGeometry(),
    ),
  );
  await prepareConvexParts([], () => true);
  await prepareConvexParts(
    [box, scenery],
    (body) => body.bodyType !== "static",
  );
  expect(load).not.toHaveBeenCalled();
});
