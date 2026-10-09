import { expect, it, vi } from "vitest";
import { BoxGeometry, Group } from "three";
import { BoxCollider, MeshCollider, RigidBody } from "../src/index.js";
import { prepareConvexParts } from "../src/colliders/decomposition.js";

const load = vi.fn();
vi.mock("../src/colliders/coacd.js", () => ({ default: load }));

it("loads CoACD only when some mesh needs decomposing", async () => {
  const box = new RigidBody().add(new BoxCollider());
  const scenery = new RigidBody({ bodyType: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new BoxGeometry(),
    ),
  );
  await prepareConvexParts(new Group(), () => true);
  await prepareConvexParts(
    new Group().add(box, scenery),
    (body) => body.bodyType !== "static",
  );
  expect(load).not.toHaveBeenCalled();
});
