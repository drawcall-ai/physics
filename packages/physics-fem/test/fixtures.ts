import { mkdir, mkdtemp, rm } from "node:fs/promises";
import { join } from "node:path";
import { afterEach, beforeEach, vi } from "vitest";
import type { FEMInput, FEMOptions } from "../src/types.js";

const reducer = vi.hoisted(() =>
  vi.fn<(input: FEMInput) => Promise<Uint8Array>>(),
);
vi.mock("../src/prepare/index.js", () => ({ prepare: reducer }));
export { reducer };

// vitecache keeps Node entries below the working directory, so each test runs in a fresh one. It lies inside the
// package so that GLTS scripts written there resolve the package's dependencies.
const root = process.cwd();
beforeEach(async () => {
  await mkdir(join(root, ".cache"), { recursive: true });
  process.chdir(await mkdtemp(join(root, ".cache", "fem-test-")));
});
afterEach(async () => {
  reducer.mockReset();
  vi.unstubAllGlobals();
  const folder = process.cwd();
  process.chdir(root);
  await rm(folder, { recursive: true, force: true });
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
