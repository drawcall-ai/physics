import { strToU8, zipSync } from "fflate";
import type { Zippable } from "fflate";

export function archive(
  files: Record<string, Uint8Array>,
): Uint8Array<ArrayBuffer> {
  const aligned: Zippable = {};
  let offset = 0;
  for (const [name, data] of Object.entries(files)) {
    const header = offset + 30 + strToU8(name).length;
    const padding = (64 - ((header + 4) % 64)) % 64;
    aligned[name] = [data, { extra: { 12345: new Uint8Array(padding) } }];
    offset = header + 4 + padding + data.length;
  }
  const result = zipSync(aligned, { level: 0 });
  return new Uint8Array(result);
}
