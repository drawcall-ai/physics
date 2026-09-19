import { execFileSync } from "node:child_process";
import { copyFile, mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { pathToFileURL } from "node:url";
import { Worker } from "node:worker_threads";
import { expect, it } from "vitest";

const decompose = `
  globalThis.fetch = () => { throw new Error("Unexpected asset fetch"); };
  const { default: createCoACD } = await import(moduleUrl);
  const api = await createCoACD();
  const result = api.decompose(
    new Float64Array([0,0,0, 1,0,0, 0,1,0, 0,0,1]),
    new Int32Array([0,2,1, 0,1,3, 0,3,2, 1,2,3]),
    0.05, -1, 50, 2000, 20, 150, 3, 256, true,
  );
  const sizes = [];
  for (const hull of result.hulls) {
    const vertices = Array.from(hull.vertices);
    if (!vertices.every(Number.isFinite)) throw new Error("Invalid hull vertices");
    sizes.push(vertices.length);
    hull.vertices.delete();
    hull.indices.delete();
  }
  result.hulls.delete();
`;

it("decomposes from a standalone JavaScript file in Node and a worker without fetching assets", async () => {
  const directory = await mkdtemp(join(tmpdir(), "coacd-embedded-"));
  try {
    const file = join(directory, "coacd.mjs");
    await copyFile(new URL("../dist/model/coacd.js", import.meta.url), file);
    const moduleUrl = pathToFileURL(file).href;
    const output = execFileSync(
      process.execPath,
      [
        "--input-type=module",
        "--eval",
        `const moduleUrl = process.argv[1]; ${decompose} console.log(JSON.stringify(sizes));`,
        moduleUrl,
      ],
      { encoding: "utf8" },
    );
    expect(output.trim().split("\n").at(-1)).toBe("[12]");

    const worker = new Worker(
      new URL(
        `data:text/javascript,${encodeURIComponent(`
      import { parentPort, workerData as moduleUrl } from "node:worker_threads";
      ${decompose}
      parentPort.postMessage(sizes);
    `)}`,
      ),
      { workerData: moduleUrl },
    );
    try {
      const sizes = await new Promise<unknown>((resolve, reject) => {
        worker.once("message", resolve);
        worker.once("error", reject);
        worker.once("exit", (code) =>
          reject(new Error(`Worker exited without a result: ${code}`)),
        );
      });
      expect(sizes).toEqual([12]);
    } finally {
      await worker.terminate();
    }
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});
