import { strFromU8, unzipSync } from "fflate";
import { USDAParser } from "three/addons/loaders/usd/USDAParser.js";
import { parseLayer, record } from "./layer.js";
import type { Layer } from "./layer.js";

export function read(input: ArrayBuffer | Uint8Array | string): {
  layer: Layer;
  assets: Record<string, unknown>;
} {
  if (typeof input === "string") {
    const layer = compose(input, {}, new Set());
    checkReferences(layer, {});
    return { layer, assets: {} };
  }
  const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
  if (bytes[0] !== 0x50 || bytes[1] !== 0x4b) return read(strFromU8(bytes));
  const files = unzipSync(bytes);
  const root = Object.keys(files)[0];
  if (!root || !files[root]) throw new Error("Empty USDZ archive");
  const assets: Record<string, unknown> = {};
  for (const [name, value] of Object.entries(files)) {
    if (/\.(usd|usda|usdc)$/i.test(name)) {
      const embedded = compose(strFromU8(value), files, new Set([name]));
      checkReferences(embedded, files);
      assets[name] = embedded;
    } else assets[name] = value;
  }
  const layer = compose(strFromU8(files[root]), files, new Set([root]));
  checkReferences(layer, files);
  return { layer, assets };
}

function checkReferences(
  layer: Layer,
  files: Record<string, Uint8Array>,
): void {
  for (const spec of Object.values(layer.specsByPath)) {
    if (spec.fields.payload !== undefined)
      throw new Error("USD payloads must be flattened before physics import");
    if (!Array.isArray(spec.fields.references)) continue;
    for (const reference of spec.fields.references) {
      if (typeof reference !== "string")
        throw new Error("Unsupported USD reference");
      const match = /@([^@]+)@/.exec(reference);
      const name = match?.[1]?.replace(/^\.\//, "");
      if (!name || !files[name])
        throw new Error(
          "USD references must target embedded visual-only layers",
        );
      const text = strFromU8(files[name]);
      if (/Physics\w+(?:API|Joint|Scene)/.test(text) || /physics:/.test(text))
        throw new Error(
          "Physics in referenced USD layers must be flattened before import",
        );
    }
  }
}

function compose(
  text: string,
  files: Record<string, Uint8Array>,
  visited: Set<string>,
): Layer {
  if (!text.startsWith("#usda 1.0"))
    throw new Error(
      "Physics USD import currently supports ASCII USDA layers; convert USDC to USDA first",
    );
  const parsed = new USDAParser().parseText(text);
  if (!record(parsed)) throw new Error("Invalid USDA document");
  const header = parsed["#usda 1.0"];
  const subLayers = record(header) ? header.subLayers : undefined;
  const result: Layer = { specsByPath: {} };
  if (subLayers !== undefined) {
    if (
      typeof subLayers !== "string" ||
      !/^\[\s*(?:@[^@]+@\s*(?:,\s*@[^@]+@\s*)*)?\]$/.test(subLayers)
    )
      throw new Error("Unsupported USD sublayer declaration");
    const names = [...subLayers.matchAll(/@([^@]+)@/g)].map(
      (match) => match[1],
    );
    for (const name of names.reverse()) {
      if (!name || visited.has(name)) throw new Error("Cyclic USD sublayers");
      const data = files[name];
      if (!data)
        throw new Error(
          `USD sublayer ${name} must be embedded in the USDZ archive`,
        );
      const nested = compose(
        strFromU8(data),
        files,
        new Set([...visited, name]),
      );
      merge(result, nested);
    }
  }
  merge(result, parseLayer(text));
  if (record(header) && header.kilogramsPerUnit !== undefined) {
    const root = result.specsByPath["/"];
    if (root) root.fields.kilogramsPerUnit = Number(header.kilogramsPerUnit);
  }
  return result;
}

function merge(target: Layer, source: Layer): void {
  for (const [path, spec] of Object.entries(source.specsByPath)) {
    const previous = target.specsByPath[path];
    const fields = { ...previous?.fields, ...spec.fields };
    if (spec.fields.typeName === "" && previous?.fields.typeName)
      fields.typeName = previous.fields.typeName;
    target.specsByPath[path] = { specType: spec.specType, fields };
  }
}
