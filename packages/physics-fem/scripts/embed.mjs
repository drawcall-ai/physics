import { cp, mkdir, readFile, readdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

const root = new URL("../", import.meta.url);
const input = resolve(
  process.argv[2] ?? fileURLToPath(new URL(".cache/assets/", root)),
);
const output = new URL("generated/", root);
const metadata = JSON.parse(
  await readFile(new URL(import.meta.resolve("pyodide/package.json")), "utf8"),
);
if (metadata.version !== "314.0.7")
  throw new Error(
    "Review embedded Pyodide loader rewrites before changing its pinned version",
  );

const pyodide = `${input}/pyodide`;
const wheels = (await readdir(pyodide))
  .filter((name) => name.endsWith(".whl"))
  .sort();
const names = ["pyodide.asm.wasm", "python_stdlib.zip", ...wheels];
const payload = Object.fromEntries(
  await Promise.all(
    names.map(async (name) => [
      name,
      (await readFile(`${pyodide}/${name}`)).toString("base64"),
    ]),
  ),
);
payload["gmsh-core.wasm"] = (
  await readFile(`${input}/gmsh/gmsh-core.wasm`)
).toString("base64");
const python = Object.fromEntries(
  await Promise.all(
    ["prepare", "reduce", "contact", "surface"].map(async (name) => [
      name,
      await readFile(`${input}/python/${name}.py`, "utf8"),
    ]),
  ),
);
const lockFileContents = await readFile(`${pyodide}/pyodide-lock.json`, "utf8");
await mkdir(new URL("gmsh/", output), { recursive: true });
await mkdir(new URL("notices/gmsh/", output), { recursive: true });
await mkdir(new URL("notices/pyodide/", output), { recursive: true });

await writeFile(
  new URL("assets.js", output),
  `const payload = ${JSON.stringify(payload)};
export const lockFileContents = ${JSON.stringify(lockFileContents)};
export const wheels = ${JSON.stringify(wheels)};
export const python = ${JSON.stringify(python)};
export function binary(name) {
  const encoded = payload[name];
  if (typeof encoded !== "string") throw new Error("Missing embedded FEM asset: " + name);
  const decoded = atob(encoded);
  const bytes = new Uint8Array(decoded.length);
  for (let i = 0; i < decoded.length; i++) bytes[i] = decoded.charCodeAt(i);
  return bytes;
}
export function getBinaryResponse(path) {
  return { binary: Promise.resolve(binary(path.slice(path.lastIndexOf("/") + 1))) };
}
`,
);
await writeFile(
  new URL("assets.d.ts", output),
  `export declare const lockFileContents: string;
export declare const wheels: readonly string[];
export declare const python: Readonly<Record<string, string>>;
export declare function binary(name: string): Uint8Array<ArrayBuffer>;
export declare function getBinaryResponse(path: string): { binary: Promise<Uint8Array<ArrayBuffer>> };
`,
);

for (const [name, dispatch, replacement] of [
  [
    "pyodide",
    "var _;f.IN_NODE?_=de:f.IN_SHELL?_=ue:_=fe;",
    "var _=embeddedBinary;",
  ],
  [
    "pyodide.asm",
    "var ze;h.IN_NODE?ze=Vn:h.IN_SHELL?ze=Jn:ze=qn;",
    "var ze=embeddedBinary;",
  ],
]) {
  let source = replace(
    await readFile(`${pyodide}/${name}.mjs`, "utf8"),
    dispatch,
    replacement,
  );
  if (name === "pyodide.asm")
    source = replace(
      source,
      'new URL("pyodide.asm.wasm",import.meta.url).href',
      '"pyodide.asm.wasm"',
    );
  source = source.replace(/\n\/\/# sourceMappingURL=.*\n?$/, "\n");
  await writeFile(
    new URL(`${name}.js`, output),
    `import { getBinaryResponse as embeddedBinary } from "./assets.js";\n${source}`,
  );
}
await writeFile(
  new URL("pyodide.d.ts", output),
  'export { loadPyodide, type PyodideInterface } from "pyodide";\n',
);
await writeFile(
  new URL("pyodide.asm.d.ts", output),
  `import type { loadPyodide } from "pyodide";
declare const create: NonNullable<NonNullable<Parameters<typeof loadPyodide>[0]>["createPyodideModule"]>;
export default create;
`,
);

for (const name of [
  "gmsh.mjs",
  "gmsh-core.mjs",
  "runtime.mjs",
  "gmsh-descriptor.mjs",
]) {
  let source = await readFile(`${input}/gmsh/${name}`, "utf8");
  if (name === "gmsh-core.mjs")
    source = replace(
      source,
      'new URL("gmsh-core.wasm",import.meta.url).href',
      '"gmsh-core.wasm"',
    );
  await writeFile(new URL(`gmsh/${name}`, output), source);
}
await writeFile(
  new URL("gmsh/gmsh.d.mts", output),
  "export declare function initialize(options: { wasmBinary: Uint8Array }): Promise<unknown>;\n",
);
await Promise.all([
  cp(`${input}/NOTICES.txt`, new URL("notices/NOTICES.txt", output)),
  cp(
    new URL("pyodide.js", output),
    new URL("notices/pyodide/pyodide.js", output),
  ),
  cp(
    new URL("pyodide.asm.js", output),
    new URL("notices/pyodide/pyodide.asm.js", output),
  ),
  cp(new URL(import.meta.url), new URL("notices/embed.mjs", output)),
  cp(`${pyodide}/licenses`, new URL("notices/pyodide/licenses/", output), {
    recursive: true,
  }),
  cp(
    `${pyodide}/pyodide-lock.json`,
    new URL("notices/pyodide/pyodide-lock.json", output),
  ),
  cp(
    `${pyodide}/provenance.json`,
    new URL("notices/pyodide/provenance.json", output),
  ),
  ...[
    "GMSH-CREDITS.txt",
    "GMSH-JS-LICENSE.txt",
    "GMSH-LICENSE.txt",
    "provenance.json",
    "build.mjs",
  ].map((name) =>
    cp(`${input}/gmsh/${name}`, new URL(`notices/gmsh/${name}`, output)),
  ),
  writeFile(
    new URL("notices/embedding.txt", output),
    "Pyodide 314.0.7 binary loading uses package-build-verified embedded bytes instead of network/filesystem requests.\n" +
      "Both getBinaryResponse dispatches were replaced; the upstream package installer and dynamic-library loader are unchanged.\n" +
      "Pyodide and Gmsh unused default WASM URL literals were replaced with filenames.\n" +
      "Build: scripts/embed.mjs. Pyodide source: https://github.com/pyodide/pyodide/tree/314.0.7\n",
  ),
]);

function replace(source, before, after) {
  if (source.split(before).length !== 2)
    throw new Error(
      `Expected exactly one embedded loader expression: ${before}`,
    );
  return source.replace(before, after);
}
