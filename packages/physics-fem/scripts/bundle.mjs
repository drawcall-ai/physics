import { cp, readdir, rm } from "node:fs/promises";
import { findPackageJSON } from "node:module";
import { fileURLToPath, pathToFileURL } from "node:url";
import { build } from "esbuild";

const root = new URL("../", import.meta.url);
const worker = fileURLToPath(new URL("dist/prepare/worker.js", root));

// Applications receive a complete worker; their bundler needs no knowledge of Python or WASM.
await build({
  entryPoints: [worker],
  outfile: worker,
  bundle: true,
  allowOverwrite: true,
  format: "esm",
  platform: "neutral",
  mainFields: ["module", "main"],
  external: ["node:*", "ws"],
  target: "es2022",
  legalComments: "eof",
});

const prepare = new URL("dist/prepare/", root);
for (const file of await readdir(prepare)) {
  if (["index.js", "index.js.map", "index.d.ts", "worker.js"].includes(file))
    continue;
  await rm(new URL(file, prepare), { recursive: true });
}
await cp(new URL("generated/notices/", root), new URL("dist/notices/", root), {
  recursive: true,
});
for (const [name, license] of [
  ["three", "LICENSE"],
  ["meshoptimizer", "LICENSE.md"],
  ["three-mesh-bvh", "LICENSE"],
]) {
  const manifest = findPackageJSON(name, import.meta.url);
  if (!manifest) throw new Error(`Cannot locate ${name}'s license`);
  await cp(
    new URL(license, pathToFileURL(manifest)),
    new URL(`dist/notices/${name}-LICENSE.txt`, root),
  );
}
