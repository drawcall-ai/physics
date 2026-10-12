/**
 * Decomposes meshes one at a time, in order, skipping those that are no longer current by
 * their turn. `run` decomposes in a worker in browsers and on the calling thread elsewhere.
 */
import { run } from "./run.js";

interface Pending {
  positions: Float64Array;
  indices: Int32Array;
  current: () => boolean;
  resolve: (parts: number[][] | undefined) => void;
  reject: (error: unknown) => void;
}

const queue: Pending[] = [];
let draining = false;

/**
 * Decomposes a closed mesh into convex parts, as flat vertex lists, or resolves undefined if
 * the mesh is no longer `current` by its turn.
 */
export function schedule(
  positions: Float64Array,
  indices: Int32Array,
  current: () => boolean,
): Promise<number[][] | undefined> {
  return new Promise((resolve, reject) => {
    queue.push({ positions, indices, current, resolve, reject });
    if (!draining) void drain();
  });
}

async function drain(): Promise<void> {
  draining = true;
  for (let next = queue.shift(); next; next = queue.shift()) {
    if (!next.current()) {
      next.resolve(undefined);
      continue;
    }
    try {
      next.resolve(await run(next.positions, next.indices));
    } catch (error) {
      next.reject(error);
    }
  }
  draining = false;
}
