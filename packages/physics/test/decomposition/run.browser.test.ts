import { afterEach, beforeEach, expect, it, vi } from "vitest";
import type {
  Reply,
  Request,
} from "../../src/colliders/decomposition/worker.js";

/** Stands in for a browser's module worker. */
class FakeWorker extends EventTarget {
  static blocked = false;
  static spawned: FakeWorker[] = [];
  readonly requests: { request: Request; transfer: Transferable[] }[] = [];
  terminated = false;
  constructor() {
    super();
    if (FakeWorker.blocked) throw new Error("blocked");
    FakeWorker.spawned.push(this);
  }
  postMessage(request: Request, transfer: Transferable[]) {
    this.requests.push({ request, transfer });
  }
  reply(data: Reply) {
    this.dispatchEvent(new MessageEvent("message", { data }));
  }
  terminate() {
    this.terminated = true;
  }
}

const part = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
const mesh = () => [new Float64Array(12), new Int32Array(12)] as const;
const hint =
  'With Vite before 8, add optimizeDeps: { exclude: ["@drawcall/physics"] }';
/** A fresh module, so each test starts without a worker. */
async function load() {
  vi.resetModules();
  return import("../../src/colliders/decomposition/run.browser.js");
}
function spawned(index: number): FakeWorker {
  const worker = FakeWorker.spawned[index];
  if (!worker) throw new Error(`Worker ${index} was not spawned`);
  return worker;
}
beforeEach(() => {
  vi.stubGlobal("Worker", FakeWorker);
  FakeWorker.blocked = false;
  FakeWorker.spawned = [];
});
afterEach(() => vi.unstubAllGlobals());

it("transfers each mesh to one worker and settles its reply", async () => {
  const { run } = await load();
  const [positions, indices] = mesh();
  const first = run(positions, indices);
  const worker = spawned(0);
  expect(worker.requests[0]?.transfer).toEqual([
    positions.buffer,
    indices.buffer,
  ]);
  worker.reply({ error: "CoACD aborted" });
  await expect(first).rejects.toThrow("CoACD aborted");
  const second = run(...mesh());
  worker.reply({ parts: [part] });
  await expect(second).resolves.toEqual([part]);
  expect(FakeWorker.spawned).toHaveLength(1);
});

it.each([
  ["error", "it could not load"],
  ["messageerror", "a reply could not be read"],
])(
  "rejects the running mesh with a hint on %s, then spawns afresh",
  async (type, reason) => {
    const { run } = await load();
    const pending = run(...mesh());
    spawned(0).dispatchEvent(new Event(type));
    await expect(pending).rejects.toThrow(
      `its worker failed (${reason}). ${hint}`,
    );
    expect(spawned(0).terminated).toBe(true);
    const next = run(...mesh());
    spawned(1).reply({ parts: [part] });
    await expect(next).resolves.toEqual([part]);
  },
);

it("rejects with a hint when the worker cannot be created", async () => {
  FakeWorker.blocked = true;
  const { run } = await load();
  await expect(run(...mesh())).rejects.toThrow(
    `its worker failed (blocked). ${hint}`,
  );
});
