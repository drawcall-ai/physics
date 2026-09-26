import { randomUUID, createHash } from "node:crypto";
import { mkdir, readFile, rename, rm, stat, writeFile } from "node:fs/promises";
import { join } from "node:path";

/**
 * Convex parts kept across runs in Node, as physics engines cache cooked collision meshes:
 * one file per key under the working directory's `node_modules/.cache`. Without a
 * `node_modules` there, nothing is cached; reads that fail are misses and writes that fail
 * are skipped, so the cache never breaks a build.
 */
async function directory(): Promise<string | undefined> {
  const modules = join(process.cwd(), "node_modules");
  const found = await stat(modules).catch(() => undefined);
  return found?.isDirectory()
    ? join(modules, ".cache", "@drawcall", "physics")
    : undefined;
}

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

export async function read(name: string): Promise<unknown> {
  const folder = await directory();
  if (!folder) return undefined;
  try {
    return JSON.parse(await readFile(join(folder, `${name}.json`), "utf8"));
  } catch {
    return undefined;
  }
}

export async function write(name: string, parts: number[][]): Promise<void> {
  const folder = await directory();
  if (!folder) return;
  const file = join(folder, `${name}.json`);
  // Written aside and renamed, so a concurrent reader never sees half a file.
  const partial = `${file}.${randomUUID()}.tmp`;
  try {
    await mkdir(folder, { recursive: true });
    await writeFile(partial, JSON.stringify(parts));
    await rename(partial, file);
  } catch {
    await rm(partial, { force: true }).catch(() => {});
  }
}
