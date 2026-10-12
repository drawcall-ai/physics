import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { existsSync, readdirSync } from "node:fs";
import { expect, it } from "vitest";

const node = (args: string[]) => promisify(execFile)(process.execPath, args);
const dist = new URL("../dist/", import.meta.url);
if (!existsSync(dist))
  throw new Error("Run pnpm build first: the test loads dist");
// The worker entry only runs inside a worker.
const modules = readdirSync(dist, { recursive: true, encoding: "utf8" })
  .map((file) => file.replaceAll("\\", "/"))
  .filter((file) => file.endsWith(".js") && !file.endsWith("worker.js"));

// An import cycle that reaches a class before it is defined fails only when the module that
// closes the cycle loads first, as `Class extends value undefined`.
it("loads each built module first in Node", async () => {
  expect(modules).toContain("joints/joint.js");
  await Promise.all(
    modules.map((file) =>
      node([
        "--input-type=module",
        "-e",
        `await import(${JSON.stringify(new URL(file, dist).href)})`,
      ]),
    ),
  );
}, 60000);
