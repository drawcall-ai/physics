/** Decomposes in one module worker, in place of `run.ts` in browser bundles. */
import type { Reply, Request } from "./worker.js";

let worker: Worker | undefined;
/** The decomposition the worker runs; the queue sends one at a time. */
let running:
  | { resolve: (parts: number[][]) => void; reject: (error: Error) => void }
  | undefined;

export function run(
  positions: Float64Array,
  indices: Int32Array,
): Promise<number[][]> {
  if (running) throw new Error("The decomposition worker is busy");
  return new Promise((resolve, reject) => {
    try {
      worker ??= spawn();
      const request: Request = { positions, indices };
      worker.postMessage(request, [positions.buffer, indices.buffer]);
      running = { resolve, reject };
    } catch (error) {
      drop();
      reject(failure(error instanceof Error ? error.message : String(error)));
    }
  });
}

function spawn(): Worker {
  const spawned = new Worker(new URL("./worker.js", import.meta.url), {
    type: "module",
  });
  spawned.addEventListener("message", ({ data }: MessageEvent<Reply>) => {
    if (!running) throw new Error("Unexpected decomposition reply");
    const { resolve, reject } = running;
    running = undefined;
    if ("parts" in data) resolve(data.parts);
    else reject(new Error(data.error));
  });
  // Some browsers report a failed load as a plain event, without a message.
  spawned.addEventListener("error", (event) =>
    fail(event.message || "it could not load"),
  );
  spawned.addEventListener("messageerror", () =>
    fail("a reply could not be read"),
  );
  return spawned;
}

/** Rejects the running decomposition and drops the worker, so the next one spawns afresh. */
function fail(reason: string): void {
  const job = running;
  running = undefined;
  drop();
  job?.reject(failure(reason));
}

function drop(): void {
  worker?.terminate();
  worker = undefined;
}

function failure(reason: string): Error {
  return new Error(
    `its worker failed (${reason}). With Vite before 8, add optimizeDeps: { exclude: ["@drawcall/physics"] } to the dev server.`,
  );
}
