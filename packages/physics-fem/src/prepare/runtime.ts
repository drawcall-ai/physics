import { loadPyodide, type PyodideInterface } from "../../generated/pyodide.js";
import createPyodideModule from "../../generated/pyodide.asm.js";
import {
  binary,
  lockFileContents,
  python as sources,
  wheels,
} from "../../generated/assets.js";
import * as geometry from "./geometry.js";

let runtime: Promise<PyodideInterface> | undefined;

export async function prepare(
  request: string,
): Promise<Uint8Array<ArrayBuffer>> {
  runtime ??= initialize().catch((error: unknown) => {
    runtime = undefined;
    throw error;
  });
  const python = await runtime;
  python.globals.set("_fem_request", request);
  try {
    const result: unknown = await python.runPythonAsync(
      "json.dumps(await prepare(json.loads(_fem_request)), allow_nan=False, separators=(',', ':'))",
    );
    if (typeof result !== "string")
      throw new Error("FEM preparation did not return JSON");
    return new TextEncoder().encode(result);
  } finally {
    python.globals.delete("_fem_request");
  }
}

async function initialize(): Promise<PyodideInterface> {
  // Upstream logs failed WASM instantiation without reliably rejecting startup.
  await WebAssembly.compile(binary("pyodide.asm.wasm"));
  const [python] = await Promise.all([
    loadPyodide({
      indexURL: "https://embedded.invalid/",
      lockFileContents,
      createPyodideModule,
      stdout: console.log,
      stderr: console.error,
    }),
    geometry.initialize(),
  ]);
  // Explicit wheel IDs avoid Node disk caching; the generated loader supplies their bytes.
  await python.loadPackage(
    wheels.map((name) => `https://embedded.invalid/${name}`),
  );
  for (const [name, source] of Object.entries(sources))
    python.FS.writeFile(`/home/pyodide/${name}.py`, source);
  python.registerJsModule("geometry", geometry);
  python.runPython("import json\nfrom prepare import prepare");
  return python;
}
