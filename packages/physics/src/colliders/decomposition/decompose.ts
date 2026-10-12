import createCoACD, { type MainModule } from "./coacd.js";
import * as cache from "./cache.js";

let coacd: Promise<MainModule> | undefined;
/** The CoACD build; bump it with the pins in scripts/build-coacd.sh to renew cached parts. */
const DECOMPOSER = "coacd-b678aa0/cdt-ec03b30/chitin-5a96998/emscripten-5.0.2";
/**
 * CoACD settings: concavity threshold, no limit on the number of parts, preprocessing and
 * sampling resolution, MCTS nodes, iterations and depth, vertices per part, and merging.
 */
const SETTINGS = [0.05, -1, 50, 2000, 20, 150, 3, 256, true] as const;

/** A closed triangle mesh's convex parts, as flat vertex lists. */
export async function decompose(
  positions: Float64Array,
  indices: Int32Array,
): Promise<number[][]> {
  // In Node the parts persist across runs, keyed by the decomposer, its settings and the mesh.
  const entry = cache.key(
    DECOMPOSER,
    JSON.stringify(SETTINGS),
    `${positions.length}/${indices.length}`,
    positions,
    indices,
  );
  const cached = entry ? await cache.read(entry) : undefined;
  if (valid(cached)) return cached;
  // Silences CoACD's progress log; errors still reach the console.
  const loading = (coacd ??= createCoACD({ print() {} }));
  let parts: number[][];
  try {
    parts = hulls(await loading, positions, indices);
  } catch (error) {
    // A failed load or an aborted run leaves no usable instance; the next run loads afresh.
    if (coacd === loading) coacd = undefined;
    throw error;
  }
  if (!valid(parts)) throw new Error("CoACD produced invalid convex parts");
  if (entry) await cache.write(entry, parts);
  return parts;
}

/** Convex parts as flat lists of at least four finite points each. */
function valid(parts: unknown): parts is number[][] {
  return (
    Array.isArray(parts) &&
    parts.length > 0 &&
    parts.every(
      (part) =>
        Array.isArray(part) &&
        part.length >= 12 &&
        part.length % 3 === 0 &&
        part.every(Number.isFinite),
    )
  );
}

function hulls(
  api: MainModule,
  positions: Float64Array,
  indices: Int32Array,
): number[][] {
  const result = api.decompose(positions, indices, ...SETTINGS);
  try {
    const parts: number[][] = [];
    for (const hull of result.hulls) {
      try {
        parts.push(Array.from(hull.vertices));
      } finally {
        hull.vertices.delete();
        hull.indices.delete();
      }
    }
    return parts;
  } finally {
    result.hulls.delete();
  }
}
