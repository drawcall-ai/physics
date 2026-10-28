import { execFileSync } from "node:child_process";
import { createHash } from "node:crypto";
import { existsSync } from "node:fs";
import { cp, mkdir, readFile, readdir, rm, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL("../", import.meta.url));
const target = join(root, ".cache/assets");
const pyodide = dirname(
  fileURLToPath(import.meta.resolve("pyodide/package.json")),
);
const cache = join(root, ".cache");
const base = "https://cdn.jsdelivr.net/pyodide/v314.0.7/full/";
const wheels = [
  {
    name: "numpy-2.4.6-cp314-cp314-pyemscripten_2026_0_wasm32.whl",
    sha256: "a292c1f5d7d8a2208cd5e94fc467604c131cabcd2fc14fed6eefde121e7fabdf",
  },
  {
    name: "scipy-1.18.0-cp314-cp314-pyemscripten_2026_0_wasm32.whl",
    sha256: "17ee329a957863516d1bb6a6aaa0c60576fac027e9cd3d43de27f58b5b599b50",
  },
  {
    name: "scikit_fem-12.0.2-py3-none-any.whl",
    url: "https://files.pythonhosted.org/packages/5b/13/d46cf943bd1b04b1b010094eb08493d6660c99c49652fa46cf7c3bf36ab5/scikit_fem-12.0.2-py3-none-any.whl",
    sha256: "34cd891f80072c0c1eb759a2371e82f0eeec91ff7c08acab96d27b6edbbe1b05",
  },
  {
    name: "trimesh-5.1.1-py3-none-any.whl",
    url: "https://files.pythonhosted.org/packages/8f/f6/fb68b8c08f864ecee8f7f7ee2f1203ed5cba2335215de69e4f621eb47db5/trimesh-5.1.1-py3-none-any.whl",
    sha256: "c5be85a3e31e9ff749f9b0aecc204e37a2cf7cca4e92cd066bb5f3d3c1127765",
  },
];
const licenses = [
  {
    name: "Pyodide",
    version: "314.0.7",
    source: "https://github.com/pyodide/pyodide/tree/314.0.7",
    url: "https://raw.githubusercontent.com/pyodide/pyodide/314.0.7/LICENSE",
    file: "PYODIDE-MPL-2.0.txt",
    sha256: "1f256ecad192880510e84ad60474eab7589218784b9a50bc7ceee34c2b91f1d5",
  },
  {
    name: "CPython",
    version: "3.14.2",
    source: "https://github.com/python/cpython/tree/v3.14.2",
    url: "https://raw.githubusercontent.com/python/cpython/v3.14.2/LICENSE",
    file: "CPYTHON-LICENSE.txt",
    sha256: "b0e25a78cffb43f4d92de8b61ccfa1f1f98ecbc22330b54b5251e7b6ba010231",
  },
];

await rm(target, { recursive: true, force: true });
await Promise.all([
  mkdir(cache, { recursive: true }),
  mkdir(join(target, "pyodide/licenses"), { recursive: true }),
  mkdir(join(target, "python"), { recursive: true }),
]);

await Promise.all([
  ...wheels.map(async ({ name, url, sha256 }) =>
    writeFile(
      join(target, "pyodide", name),
      await download(name, url ?? `${base}${name}`, sha256),
    ),
  ),
  ...licenses.map(async ({ file, url, sha256 }) =>
    writeFile(
      join(target, "pyodide/licenses", file),
      await download(file, url, sha256),
    ),
  ),
]);
await writeFile(
  join(target, "pyodide/provenance.json"),
  JSON.stringify(licenses, null, 2) + "\n",
);

const gmsh = join(cache, "gmsh");
const manifestPath = join(gmsh, "provenance.json");
const script = join(root, "scripts/gmsh.mjs");
const manifest = existsSync(manifestPath)
  ? JSON.parse(await readFile(manifestPath, "utf8"))
  : undefined;
if (
  !manifest ||
  manifest.files["build.mjs"] !== hash(await readFile(script)) ||
  !Object.keys(manifest.files).every((name) => existsSync(join(gmsh, name)))
) {
  execFileSync(
    process.execPath,
    [script, process.argv[2] ?? join(cache, "gmsh-build"), gmsh],
    { stdio: "inherit" },
  );
}
const built = JSON.parse(await readFile(manifestPath, "utf8"));
for (const [name, sha256] of Object.entries(built.files)) {
  if (hash(await readFile(join(gmsh, name))) !== sha256)
    throw new Error(`FEM Gmsh checksum mismatch: ${name}`);
}

await Promise.all([
  ...[
    "pyodide.mjs",
    "pyodide.asm.mjs",
    "pyodide.asm.wasm",
    "pyodide-lock.json",
    "python_stdlib.zip",
  ].map((name) => cp(join(pyodide, name), join(target, "pyodide", name))),
  ...(await readdir(join(root, "src/prepare")))
    .filter((name) => name.endsWith(".py"))
    .map((name) =>
      cp(join(root, "src/prepare", name), join(target, "python", name)),
    ),
  cp(gmsh, join(target, "gmsh"), { recursive: true }),
  cp(join(root, "scripts/NOTICES.txt"), join(target, "NOTICES.txt")),
]);
function hash(bytes) {
  return createHash("sha256").update(bytes).digest("hex");
}

async function download(name, url, sha256) {
  const cached = join(cache, name);
  let bytes;
  if (existsSync(cached)) {
    bytes = await readFile(cached);
  } else {
    const response = await fetch(url);
    if (!response.ok)
      throw new Error(`Cannot download ${name}: HTTP ${response.status}`);
    bytes = new Uint8Array(await response.arrayBuffer());
  }
  if (hash(bytes) !== sha256)
    throw new Error(`FEM dependency checksum mismatch: ${name}`);
  await writeFile(cached, bytes);
  return bytes;
}
