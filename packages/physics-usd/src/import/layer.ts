import { USDAParser } from "three/addons/loaders/usd/USDAParser.js";

export interface Spec {
  specType: number;
  fields: Record<string, unknown>;
}
export interface Layer {
  specsByPath: Record<string, Spec>;
}

export function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

export function parseLayer(text: string): Layer {
  for (const line of text.split("\n")) {
    if ((line.match(/physics:/g)?.length ?? 0) > 1)
      throw new Error("Physics USD import requires one property per line");
    if (
      /^\s*(?:def|over|class)\b.*\(.*\)/.test(line) ||
      /^\s*(?:(?:def|over|class)\b.*)?\{.*\}/.test(line)
    ) {
      throw new Error(
        "Physics USD import requires multiline prim bodies and metadata; expand inline USDA syntax first",
      );
    }
  }
  const parsed = new USDAParser().parseData(text);
  if (!record(parsed) || !record(parsed.specsByPath))
    throw new Error("Invalid USD layer");
  const specsByPath: Record<string, Spec> = {};
  for (const [path, value] of Object.entries(parsed.specsByPath)) {
    if (
      !record(value) ||
      typeof value.specType !== "number" ||
      !record(value.fields)
    ) {
      throw new Error(`Invalid USD spec at ${path}`);
    }
    specsByPath[path] = { specType: value.specType, fields: value.fields };
  }
  const tree = new USDAParser().parseText(text);
  const visit = (node: unknown, parent: string) => {
    if (!record(node)) return;
    for (const [key, value] of Object.entries(node)) {
      if (key === "variants" || key.startsWith("variantSet "))
        throw new Error("USD variants must be flattened before physics import");
      if (key.startsWith("over ") || key.startsWith("class "))
        throw new Error(
          "Physics USD import requires flattened def prims; over/class specs are unsupported",
        );
      const match = /^def\s+(?:\w+\s+)?"([^"]+)"$/.exec(key);
      if (!match || !record(value)) continue;
      const path = `${parent}/${match[1]}`;
      const spec = specsByPath[path];
      if (spec && typeof value.displayName === "string") {
        const displayName: unknown = JSON.parse(value.displayName);
        if (typeof displayName !== "string")
          throw new Error(`Invalid displayName on ${path}`);
        spec.fields.displayName = displayName;
      }
      for (const axis of ["angular", "linear"]) {
        const name = `drive:${axis}:physics:maxForce`;
        // Three's parser turns USDA's positive infinity token into NaN.
        if (value[`float ${name}`] !== "inf") continue;
        const property = specsByPath[`${path}.${name}`];
        if (!property) throw new Error(`Missing USD property ${path}.${name}`);
        property.fields.default = Infinity;
      }
      visit(value, path);
    }
  };
  visit(tree, "");
  return { specsByPath };
}

export function schemas(layer: Layer, path: string): string[] {
  const raw = layer.specsByPath[`${path}.apiSchemas`]?.fields.default;
  if (raw === undefined) return [];
  const value: unknown = typeof raw === "string" ? JSON.parse(raw) : raw;
  if (!Array.isArray(value) || !value.every((item) => typeof item === "string"))
    throw new Error(`Invalid API schemas on ${path}`);
  return value;
}

export function attribute(layer: Layer, path: string, name: string): unknown {
  return layer.specsByPath[`${path}.${name}`]?.fields.default;
}

export function numeric(
  layer: Layer,
  path: string,
  name: string,
  fallback: number,
): number {
  const value = attribute(layer, path, name);
  if (value === undefined) return fallback;
  if (typeof value !== "number" || !Number.isFinite(value))
    throw new Error(`Expected finite ${path}.${name}`);
  return value;
}

export function boolean(
  layer: Layer,
  path: string,
  name: string,
  fallback: boolean,
): boolean {
  const value = attribute(layer, path, name);
  if (value === undefined) return fallback;
  if (value === true || value === "true" || value === 1) return true;
  if (value === false || value === "false" || value === 0) return false;
  throw new Error(`Expected boolean ${path}.${name}`);
}

export function token(
  layer: Layer,
  path: string,
  name: string,
  fallback: string,
): string {
  const value = attribute(layer, path, name);
  if (value === undefined) return fallback;
  if (typeof value !== "string")
    throw new Error(`Expected token ${path}.${name}`);
  return value;
}

export function numbers(
  layer: Layer,
  path: string,
  name: string,
): number[] | undefined {
  const value = attribute(layer, path, name);
  if (value === undefined) return undefined;
  if (
    !Array.isArray(value) ||
    !value.every((item) => typeof item === "number" && Number.isFinite(item))
  ) {
    throw new Error(`Expected numeric array ${path}.${name}`);
  }
  return value;
}

export function target(
  layer: Layer,
  path: string,
  name: string,
): string | undefined {
  const value = layer.specsByPath[`${path}.${name}`]?.fields.targetPaths;
  if (value === undefined) return undefined;
  if (
    !Array.isArray(value) ||
    value.length !== 1 ||
    typeof value[0] !== "string"
  )
    throw new Error(`Expected one target ${path}.${name}`);
  return value[0];
}
