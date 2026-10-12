import { expect, it, vi } from "vitest";
import { schedule } from "../../src/colliders/decomposition/thread.js";

/** Runs that settle only when the test says so. */
const runs: {
  resolve: (parts: number[][]) => void;
  reject: (error: Error) => void;
}[] = [];
vi.mock("../../src/colliders/decomposition/run.js", () => ({
  run: () => new Promise((resolve, reject) => runs.push({ resolve, reject })),
}));

const part = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
const mesh = (current = () => true) =>
  [new Float64Array(12), new Int32Array(12), current] as const;
function run(index: number) {
  const pending = runs[index];
  if (!pending) throw new Error(`Run ${index} has not started`);
  return pending;
}

it("runs one mesh at a time, in order, skipping those no longer current", async () => {
  let current = true;
  const first = schedule(...mesh());
  const skipped = schedule(...mesh(() => current));
  const last = schedule(...mesh());
  expect(runs).toHaveLength(1);
  current = false;
  run(0).reject(new Error("CoACD aborted"));
  await expect(first).rejects.toThrow("CoACD aborted");
  await expect(skipped).resolves.toBeUndefined();
  expect(runs).toHaveLength(2);
  run(1).resolve([part]);
  await expect(last).resolves.toEqual([part]);
});
