/**
 * Convex parts kept across runs in Node, as physics engines cache cooked collision meshes:
 * one file per key under the working directory's `node_modules/.cache`. Without a
 * `node_modules` there, nothing is cached; reads that fail are misses and writes that fail
 * are skipped, so the cache never breaks a build.
 *
 * Node's modules come from `process.getBuiltinModule` rather than imports, so browser
 * bundlers see none. Browsers lack it and cache nothing.
 */
const runtime = globalThis.process as Partial<NodeJS.Process> | undefined;
const node = runtime?.getBuiltinModule && {
  crypto: runtime.getBuiltinModule("node:crypto"),
  files: runtime.getBuiltinModule("node:fs/promises"),
  path: runtime.getBuiltinModule("node:path"),
};

/** The entry for `inputs`; none where nothing can be cached. */
export function key(
  ...inputs: (ArrayBufferView | string)[]
): string | undefined {
  if (!node) return undefined;
  const hash = node.crypto.createHash("sha256");
  for (const input of inputs)
    hash.update(
      typeof input === "string"
        ? input
        : new Uint8Array(input.buffer, input.byteOffset, input.byteLength),
    );
  return hash.digest("hex");
}

async function directory(): Promise<string | undefined> {
  if (!node) return undefined;
  const modules = node.path.join(process.cwd(), "node_modules");
  const found = await node.files.stat(modules).catch(() => undefined);
  return found?.isDirectory()
    ? node.path.join(modules, ".cache", "@drawcall", "physics")
    : undefined;
}

export async function read(name: string): Promise<unknown> {
  const folder = await directory();
  if (!node || !folder) return undefined;
  try {
    const file = node.path.join(folder, `${name}.json`);
    return JSON.parse(await node.files.readFile(file, "utf8"));
  } catch {
    return undefined;
  }
}

export async function write(name: string, parts: number[][]): Promise<void> {
  const folder = await directory();
  if (!node || !folder) return;
  const { crypto, files, path } = node;
  const file = path.join(folder, `${name}.json`);
  // Written aside and renamed, so a concurrent reader never sees half a file.
  const partial = `${file}.${crypto.randomUUID()}.tmp`;
  try {
    await files.mkdir(folder, { recursive: true });
    await files.writeFile(partial, JSON.stringify(parts));
    await files.rename(partial, file);
  } catch {
    await files.rm(partial, { force: true }).catch(() => {});
  }
}
