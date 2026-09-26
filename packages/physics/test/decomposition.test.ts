import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { mkdtemp } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import {
  BoxGeometry,
  ExtrudeGeometry,
  PlaneGeometry,
  Shape,
  Vector3,
} from "three";
import {
  MeshCollider,
  RigidBody,
  convexParts,
  prepareConvexParts,
  registry,
} from "../src/index.js";

// Without node_modules in the working directory, every test decomposes afresh.
beforeEach(async () => {
  vi.spyOn(process, "cwd").mockReturnValue(
    await mkdtemp(join(tmpdir(), "physics-decomposition-")),
  );
});

afterEach(() => {
  registry.clear();
  vi.restoreAllMocks();
});

/** An L-shaped prism: concave, so it decomposes into more than one part. */
function ell() {
  const outline = new Shape()
    .moveTo(0, 0)
    .lineTo(1, 0)
    .lineTo(1, 0.4)
    .lineTo(0.4, 0.4)
    .lineTo(0.4, 1)
    .lineTo(0, 1)
    .closePath();
  return new ExtrudeGeometry(outline, { depth: 0.4, bevelEnabled: false });
}

function trimesh(geometry: BoxGeometry | ExtrudeGeometry | PlaneGeometry) {
  return new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry);
}

it("decomposes each selected mesh once and scales its parts on request", async () => {
  const geometry = ell();
  const first = trimesh(geometry);
  const second = trimesh(geometry);
  const body = new RigidBody().add(first, second);
  const needs = vi.fn(() => true);
  await prepareConvexParts([body], needs);
  expect(needs).toHaveBeenCalledTimes(1);
  const parts = convexParts(first, new Vector3(1, 1, 1));
  expect(parts?.length).toBeGreaterThan(1);
  const doubled = convexParts(second, new Vector3(2, 2, 2));
  expect(doubled?.[0]?.[0]).toBeCloseTo(2 * parts![0]![0]!);
});

it("forgets the parts of a mesh edited after decomposition", async () => {
  const geometry = ell();
  const collider = trimesh(geometry);
  await prepareConvexParts([new RigidBody().add(collider)], () => true);
  geometry.getAttribute("position").setX(0, 5);
  expect(convexParts(collider, new Vector3(1, 1, 1))).toBeUndefined();
});

it("names the body and collider of a mesh it cannot decompose", async () => {
  const body = new RigidBody();
  body.name = "sheet";
  const collider = trimesh(new PlaneGeometry());
  collider.name = "surface";
  body.add(collider);
  await expect(prepareConvexParts([body], () => true)).rejects.toThrow(
    "Convex decomposition failed for sheet/surface",
  );
});
