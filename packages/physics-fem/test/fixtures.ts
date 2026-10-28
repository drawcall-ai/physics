import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import { pathToFileURL } from "node:url";
import { afterEach, vi } from "vitest";
import { GLTSLoader } from "@drawcall/glts";
import { LoadingManager, type Object3D } from "three";
import { SoftBody } from "@drawcall/physics";
import type { FEMLoader } from "../src/loader.js";
import type { FEMInput, FEMOptions } from "../src/types.js";

const contexts: GLTSLoader[] = [];
let contextURL: string | undefined;
const folders: string[] = [];
const reducer = vi.hoisted(() =>
  vi.fn<(input: FEMInput) => Promise<Uint8Array>>(),
);
vi.mock("../src/prepare/index.js", () => ({ prepare: reducer }));
export { reducer };

afterEach(async () => {
  for (const loader of contexts.splice(0)) loader.dispose();
  contextURL = undefined;
  reducer.mockReset();
  vi.unstubAllGlobals();
  await Promise.all(
    folders
      .splice(0)
      .map((folder) => rm(folder, { recursive: true, force: true })),
  );
});

export const options: FEMOptions = {
  material: {
    density: 1000,
    elasticity: { model: "linear", youngModulus: 1000, poissonRatio: 0.3 },
  },
  mount: { min: [-1, -1, -1], max: [1, 0, 1] },
  contact: { min: [-1, 0, -1], max: [1, 1, 1] },
  mesh: { cellSize: 0.2, order: 1 },
};

export async function folder(base = tmpdir()): Promise<string> {
  await mkdir(base, { recursive: true });
  const value = await mkdtemp(join(base, "fem-test-"));
  folders.push(value);
  return value;
}

export function sourceFolder(): Promise<string> {
  return folder(join(process.cwd(), ".cache"));
}

export function fixture(
  input: FEMInput,
  changes: Record<string, unknown> = {},
): Uint8Array {
  const nodes = input.positions.length / 3;
  const coordinate = nodes - 1;
  const basis = Array.from({ length: input.positions.length * 3 }, (_, i) => {
    const row = Math.floor(i / 3),
      column = i % 3;
    return row === coordinate * 3 + column ? 1 : 0;
  });
  const data = {
    version: 1,
    material: input.material,
    positions: input.positions,
    triangles: input.triangles,
    mount: Array.from({ length: nodes - 1 }, (_, i) => i),
    coordinates: [coordinate],
    basis,
    mass: Array.from({ length: 36 }, (_, i) => (i % 7 === 0 ? 1 : 0)),
    stiffness: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    gravity: Array(18).fill(0),
    renderPositions: input.positions,
    renderTriangles: input.triangles,
    contactRegion: input.contact,
    mountRegion: input.mount,
    renderBasis: basis,
  };
  return new TextEncoder().encode(JSON.stringify({ ...data, ...changes }));
}

export async function loadObject(
  loader: FEMLoader,
  object: Object3D,
  options: FEMOptions,
): Promise<SoftBody> {
  if (!contextURL) {
    const directory = await sourceFolder();
    contextURL = pathToFileURL(join(directory, "context.glts")).href;
    await writeFile(
      new URL(contextURL),
      `
      import { scene } from "@drawcall/glts";
      import { load } from "native";
      await Promise.resolve();
      scene.userData.body = await load();
    `,
    );
  }
  let result: Promise<SoftBody> | undefined;
  const host = new GLTSLoader(new LoadingManager(), {
    modules: {
      native: {
        load: () => (result = loader.loadAsync({ object, ...options })),
      },
    },
  });
  contexts.push(host);
  let scene;
  try {
    scene = await host.loadAsync(contextURL);
  } catch (error) {
    // Preserve FEM diagnostics instead of the GLTS execution wrapper.
    if (result) await result;
    throw error;
  }
  const body: unknown = scene.userData.body;
  if (!(body instanceof SoftBody))
    throw new Error("Missing GLTS-prepared body");
  return body;
}
