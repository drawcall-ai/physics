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
import { MeshCollider, RigidBody } from "../../src/index.js";
import { convexParts } from "../../src/backend.js";
import { settle } from "../../src/colliders/decomposition/parts.js";
import { decompose as coacd, part } from "./fake-coacd.js";

vi.mock(
  "../../src/colliders/decomposition/coacd.js",
  () => import("./fake-coacd.js"),
);

let root: string;
const cache = () =>
  join(root, "node_modules", ".cache", "@drawcall", "physics");
beforeEach(async () => {
  root = await mkdtemp(join(tmpdir(), "physics-cache-"));
  vi.spyOn(process, "cwd").mockReturnValue(root);
  coacd.mockClear();
});
afterEach(async () => {
  vi.restoreAllMocks();
  await chmod(root, 0o700).catch(() => {});
  await rm(root, { recursive: true, force: true });
});

function box() {
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    new BoxGeometry(),
  );
  return { body: new RigidBody().add(collider), collider };
}
const prepare = ({ body }: { body: RigidBody }) => settle(body, () => true);

it("reads an identical mesh's parts back from disk", async () => {
  await mkdir(join(root, "node_modules"));
  await prepare(box());
  const again = box();
  await prepare(again);
  expect(coacd).toHaveBeenCalledTimes(1);
  expect(convexParts(again.body, again.collider, new Vector3(1, 1, 1))).toEqual(
    [part],
  );
});

it("decomposes again when a cached entry is not valid parts", async () => {
  await mkdir(join(root, "node_modules"));
  await prepare(box());
  for (const file of await readdir(cache()))
    await writeFile(join(cache(), file), "[[1, 2]]");
  await prepare(box());
  expect(coacd).toHaveBeenCalledTimes(2);
});

// Permissions do not stop root, nor apply on Windows.
it.skipIf(process.getuid?.() === 0 || process.platform === "win32")(
  "still builds when the cache cannot be written",
  async () => {
    await mkdir(join(root, "node_modules"), { mode: 0o500 });
    await prepare(box());
    expect(coacd).toHaveBeenCalledTimes(1);
  },
);

it("caches nothing without a node_modules directory", async () => {
  await prepare(box());
  expect(await readdir(root)).toEqual([]);
});
