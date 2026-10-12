import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import {
  BoxGeometry,
  BufferGeometry,
  ExtrudeGeometry,
  InterleavedBuffer,
  InterleavedBufferAttribute,
  PlaneGeometry,
  Shape,
  Vector3,
} from "three";
import { MeshCollider, RigidBody } from "../../src/index.js";
import { convexParts, vertices } from "../../src/backend.js";
import { settle } from "../../src/colliders/decomposition/parts.js";

// These tests run the real CoACD. Without node_modules in the working directory, every test decomposes afresh.
let root: string;
beforeEach(async () => {
  root = await mkdtemp(join(tmpdir(), "physics-decomposition-"));
  vi.spyOn(process, "cwd").mockReturnValue(root);
});

afterEach(async () => {
  vi.restoreAllMocks();
  await rm(root, { recursive: true, force: true });
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

function trimesh(geometry: BufferGeometry) {
  return new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry);
}

it("scales the parts of a mesh for each collider sharing it", async () => {
  const geometry = ell();
  const first = trimesh(geometry);
  const second = trimesh(geometry);
  const body = new RigidBody().add(first, second);
  await settle(body, () => true);
  const parts = convexParts(body, first, new Vector3(1, 1, 1));
  expect(parts.length).toBeGreaterThan(1);
  const doubled = convexParts(body, second, new Vector3(2, 2, 2));
  expect(doubled.flat()).toEqual(parts.flat().map((value) => value * 2));
});

it("stands in the hull for a mesh edited after decomposition until it is decomposed again", async () => {
  const geometry = ell();
  const collider = trimesh(geometry);
  const body = new RigidBody().add(collider);
  await settle(body, () => true);
  geometry.translate(1, 0, 0);
  const scale = new Vector3(1, 2, 1);
  const hull = vertices(geometry).map((value, i) =>
    i % 3 === 1 ? value * 2 : value,
  );
  expect(convexParts(body, collider, scale)).toEqual([hull]);
  await settle(body, () => true);
  expect(convexParts(body, collider, scale).length).toBeGreaterThan(1);
});

it("names the body and collider of a mesh it cannot decompose", async () => {
  const body = new RigidBody();
  body.name = "sheet";
  const collider = trimesh(new PlaneGeometry());
  collider.name = "surface";
  body.add(collider);
  await expect(settle(body, () => true)).rejects.toThrow(
    "Convex decomposition failed for sheet/surface",
  );
  expect(() => convexParts(body, collider, new Vector3(1, 1, 1))).toThrow(
    "Convex decomposition failed for sheet/surface: the mesh is not closed",
  );
});

it("decomposes interleaved positions, ignoring unrelated channels", async () => {
  const box = new BoxGeometry();
  const plain = box.getAttribute("position");
  // A fourth channel of nines: read as a coordinate, it would stretch the parts.
  const data = new InterleavedBuffer(
    new Float32Array(plain.count * 4).fill(9),
    4,
  );
  for (let i = 0; i < plain.count; i++)
    data.array.set([plain.getX(i), plain.getY(i), plain.getZ(i)], i * 4);
  const geometry = new BufferGeometry()
    .setAttribute("position", new InterleavedBufferAttribute(data, 3, 0))
    .setIndex(box.index);
  const collider = trimesh(geometry);
  const body = new RigidBody().add(collider);
  await settle(body, () => true);
  const points = convexParts(body, collider, new Vector3(1, 1, 1)).flat();
  expect(Math.max(...points.map(Math.abs))).toBeCloseTo(0.5);
});
