import { afterEach, beforeEach, expect, it, vi } from "vitest";
import {
  chmod,
  mkdir,
  mkdtemp,
  readdir,
  rm,
  writeFile,
} from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { BoxGeometry, Vector3 } from "three";
import {
  MeshCollider,
  RigidBody,
  convexParts,
  prepareConvexParts,
  registry,
} from "../src/index.js";

const hull = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
const decompose = vi.fn(() => ({
  hulls: Object.assign(
    [
      {
        vertices: Object.assign([...hull], { delete() {} }),
        indices: { delete() {} },
      },
    ],
    { delete() {} },
  ),
}));
vi.mock("../src/coacd.js", () => ({ default: async () => ({ decompose }) }));

let root: string;
const cache = () =>
  join(root, "node_modules", ".cache", "@drawcall", "physics");
beforeEach(async () => {
  root = await mkdtemp(join(tmpdir(), "physics-cache-"));
  vi.spyOn(process, "cwd").mockReturnValue(root);
  decompose.mockClear();
});
afterEach(async () => {
  registry.clear();
  vi.restoreAllMocks();
  await chmod(root, 0o700).catch(() => {});
  await rm(root, { recursive: true, force: true });
});

function box() {
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    new BoxGeometry(),
  );
  new RigidBody().add(collider);
  return collider;
}
const prepare = (collider: MeshCollider) =>
  prepareConvexParts([collider.parent], () => true);

it("reads an identical mesh's parts back from disk", async () => {
  await mkdir(join(root, "node_modules"));
  await prepare(box());
  const again = box();
  await prepare(again);
  expect(decompose).toHaveBeenCalledTimes(1);
  expect(convexParts(again, new Vector3(1, 1, 1))).toEqual([hull]);
});

it("decomposes again when a cached entry is not valid parts", async () => {
  await mkdir(join(root, "node_modules"));
  await prepare(box());
  for (const file of await readdir(cache()))
    await writeFile(join(cache(), file), "[[1, 2]]");
  await prepare(box());
  expect(decompose).toHaveBeenCalledTimes(2);
});

it("still builds when the cache cannot be written", async () => {
  await mkdir(join(root, "node_modules"), { mode: 0o500 });
  await prepare(box());
  expect(decompose).toHaveBeenCalledTimes(1);
});

it("caches nothing without a node_modules directory", async () => {
  await prepare(box());
  expect(await readdir(root)).toEqual([]);
});
