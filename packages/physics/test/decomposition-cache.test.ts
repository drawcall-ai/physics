import { afterEach, expect, it, vi } from "vitest";
import { BoxGeometry } from "three";
import {
  MeshCollider,
  RigidBody,
  prepareConvexParts,
  registry,
} from "../src/index.js";

const hull = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
const decompose = vi.fn(() => ({
  hulls: Object.assign(
    [
      {
        vertices: Object.assign(hull, { delete() {} }),
        indices: { delete() {} },
      },
    ],
    { delete() {} },
  ),
}));
vi.mock("../src/coacd.js", () => ({ default: async () => ({ decompose }) }));

afterEach(() => registry.clear());

it("keeps convex parts on disk for identical meshes in later runs", async () => {
  // A mesh no earlier test run has cached.
  const offset = Math.random();
  const body = () =>
    new RigidBody().add(
      new MeshCollider({ approximation: "trimesh" }).setGeometry(
        new BoxGeometry().translate(offset, 0, 0),
      ),
    );
  await prepareConvexParts([body()], () => true);
  expect(decompose).toHaveBeenCalledTimes(1);
  await prepareConvexParts([body()], () => true);
  expect(decompose).toHaveBeenCalledTimes(1);
});
