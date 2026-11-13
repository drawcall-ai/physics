import type { SoftBody } from "@drawcall/physics";
import { build, reduce } from "./body.js";
import { cache } from "./cache.js";
import { input } from "./input.js";
import { source } from "./source.js";
import type { FEMRequest } from "./types.js";

/**
 * Prepares static, closed solids as reduced SoftBodies and caches them. The vitecache plugin warms each literal
 * `new CachedFEMLoader().loadAsync({ ... })` in Node, so browsers read the prepared result.
 * The created object becomes the body's visual and is caller-owned. Scale and local mesh transforms are baked.
 */
export class CachedFEMLoader {
  async loadAsync({ object, ...options }: FEMRequest): Promise<SoftBody> {
    const snapshot = source(await object(), true);
    const recipe = input(snapshot, options);
    const entry = cache(recipe);
    const cached = await entry.load();
    const bytes = cached ?? (await reduce(recipe));
    const body = build(snapshot, recipe, bytes);
    if (cached === undefined) await entry.save(bytes);
    return body;
  }
}
