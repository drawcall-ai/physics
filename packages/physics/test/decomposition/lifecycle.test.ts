import { beforeEach, expect, it, vi } from "vitest";
import { BoxGeometry, Group, Vector3 } from "three";
import { MeshCollider, RigidBody } from "../../src/index.js";
import { convexParts, partsVersion } from "../../src/backend.js";
import { settle } from "../../src/colliders/decomposition/parts.js";
import { decompose as coacd, part } from "./fake-coacd.js";

vi.mock(
  "../../src/colliders/decomposition/coacd.js",
  () => import("./fake-coacd.js"),
);
// Parts cached on disk would skip CoACD.
vi.mock("../../src/colliders/decomposition/cache.js", () => ({
  key: () => undefined,
}));
beforeEach(() => coacd.mockClear());

function box() {
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    new BoxGeometry(),
  );
  collider.name = "crate";
  const body = new RigidBody().add(collider);
  body.name = "cart";
  return { body, collider, geometry: collider.geometry };
}
const unit = new Vector3(1, 1, 1);
const all = () => true;

it("stands in the hull until the parts arrive, which changes the parts version", async () => {
  const { body, collider, geometry } = box();
  const version = partsVersion(geometry);
  const [hull] = convexParts(body, collider, unit);
  expect(hull).toHaveLength(geometry.getAttribute("position").count * 3);
  expect(partsVersion(geometry)).toBe(version);
  await settle(body, all);
  expect(coacd).toHaveBeenCalledTimes(1);
  expect(convexParts(body, collider, unit)).toEqual([part]);
  expect(partsVersion(geometry)).not.toBe(version);
});

it("shares one decomposition among concurrent callers", async () => {
  const { body, collider } = box();
  const first = settle(body, all);
  await settle(body, all);
  expect(convexParts(body, collider, unit)).toEqual([part]);
  await first;
  expect(coacd).toHaveBeenCalledTimes(1);
});

it("waits for meshes that join while it waits", async () => {
  const root = new Group().add(box().body);
  const done = settle(root, all);
  const later = box();
  root.add(later.body);
  await done;
  expect(convexParts(later.body, later.collider, unit)).toEqual([part]);
});

it("waits for the current version of a mesh edited while it waits", async () => {
  const { body, collider, geometry } = box();
  const done = settle(body, all);
  geometry.translate(1, 0, 0);
  await done;
  expect(coacd).toHaveBeenCalledTimes(2);
  expect(convexParts(body, collider, unit)).toEqual([part]);
});

it("decomposes only the latest version of a mesh edited while others wait", async () => {
  const { body, collider, geometry } = box();
  for (let i = 0; i < 5; i++) {
    convexParts(body, collider, unit);
    geometry.translate(1, 0, 0);
  }
  await settle(body, all);
  // The first version was in flight; the next four were replaced before their turn.
  expect(coacd).toHaveBeenCalledTimes(2);
  expect(convexParts(body, collider, unit)).toEqual([part]);
});

it("ignores meshes that are not needed, even failed ones", async () => {
  coacd.mockImplementationOnce(() => {
    throw new Error("out of memory");
  });
  const { body, collider } = box();
  await expect(settle(body, all)).rejects.toThrow("out of memory");
  await settle(body, () => false);
  expect(() => convexParts(body, collider, unit)).toThrow("out of memory");
});

it("records a failed decomposition until the mesh is edited", async () => {
  coacd.mockImplementationOnce(() => {
    throw new Error("out of memory");
  });
  const { body, collider, geometry } = box();
  const version = partsVersion(geometry);
  convexParts(body, collider, unit);
  const failed = "Convex decomposition failed for cart/crate: out of memory";
  await expect(settle(body, all)).rejects.toThrow(failed);
  expect(() => convexParts(body, collider, unit)).toThrow(failed);
  expect(partsVersion(geometry)).not.toBe(version);
  geometry.getAttribute("position").needsUpdate = true;
  expect(convexParts(body, collider, unit)).toHaveLength(1);
  await settle(body, all);
  expect(convexParts(body, collider, unit)).toEqual([part]);
});

it("fails a mesh that is not closed at once", () => {
  const { body, collider, geometry } = box();
  geometry.setIndex(Array.from(geometry.index?.array ?? []).slice(3));
  expect(() => convexParts(body, collider, unit)).toThrow(
    "Convex decomposition failed for cart/crate: the mesh is not closed",
  );
  expect(coacd).not.toHaveBeenCalled();
});
