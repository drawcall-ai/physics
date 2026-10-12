import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import {
  BoxGeometry,
  BufferGeometry,
  InterleavedBuffer,
  InterleavedBufferAttribute,
  Vector3,
} from "three";
import { MeshCollider, RigidBody } from "../../src/index.js";
import { convexParts } from "../../src/backend.js";
import { settle } from "../../src/colliders/decomposition/parts.js";

// Runs the real CoACD. Without node_modules in the working directory, nothing is cached.
let root: string;
beforeEach(async () => {
  root = await mkdtemp(join(tmpdir(), "physics-decomposition-"));
  vi.spyOn(process, "cwd").mockReturnValue(root);
});

afterEach(async () => {
  vi.restoreAllMocks();
  await rm(root, { recursive: true, force: true });
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
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    geometry,
  );
  const body = new RigidBody().add(collider);
  await settle(body, () => true);
  const points = convexParts(body, collider, new Vector3(1, 1, 1)).flat();
  expect(Math.max(...points.map(Math.abs))).toBeCloseTo(0.5);
});
