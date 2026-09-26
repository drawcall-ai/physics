import { createHash } from "node:crypto";
import { mkdir, readFile, rename, writeFile } from "node:fs/promises";
import { join } from "node:path";

/**
 * Convex parts kept across runs in Node, as physics engines cache cooked collision meshes:
 * one file per mesh and decomposition settings under the conventional
 * `node_modules/.cache` directory of the working directory.
 */
const directory = join(
  process.cwd(),
  "node_modules",
  ".cache",
  "@drawcall",
  "physics",
);

export function key(...inputs: (ArrayBufferView | string)[]): string {
  const hash = createHash("sha256");
  for (const input of inputs)
    hash.update(
      typeof input === "string"
        ? input
        : new Uint8Array(input.buffer, input.byteOffset, input.byteLength),
    );
  return hash.digest("hex");
}

export async function read(name: string): Promise<number[][] | undefined> {
  try {
    return JSON.parse(await readFile(join(directory, `${name}.json`), "utf8"));
  } catch {
    return undefined;
  }
}

export async function write(name: string, parts: number[][]): Promise<void> {
  await mkdir(directory, { recursive: true });
  // Written aside and renamed, so a concurrent reader never sees half a file.
  const file = join(directory, `${name}.json`);
  const partial = `${file}.${process.pid}.tmp`;
  await writeFile(partial, JSON.stringify(parts));
  await rename(partial, file);
}
