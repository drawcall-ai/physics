import { expect, it, vi } from "vitest";
import { BoxGeometry, Group } from "three";
import { BoxCollider, MeshCollider, RigidBody } from "../../src/index.js";
import { settle } from "../../src/colliders/decomposition/parts.js";
import load from "./fake-coacd.js";

vi.mock(
  "../../src/colliders/decomposition/coacd.js",
  () => import("./fake-coacd.js"),
);

it("loads CoACD only when some mesh needs decomposing", async () => {
  const box = new RigidBody().add(new BoxCollider());
  const scenery = new RigidBody({ bodyType: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(
      new BoxGeometry(),
    ),
  );
  await settle(new Group(), () => true);
  await settle(
    new Group().add(box, scenery),
    (body) => body.bodyType !== "static",
  );
  expect(load).not.toHaveBeenCalled();
});
