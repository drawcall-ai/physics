import { CacheEntry } from "vitecache";
import type { FEMInput } from "./types.js";
import { REDUCER_VERSION } from "./types.js";

export async function cache(
  input: FEMInput,
  basePath?: string,
): Promise<CacheEntry> {
  const { density, elasticity } = input.material;
  const encoded = new TextEncoder().encode(
    JSON.stringify({
      positions: input.positions,
      triangles: input.triangles,
      material: {
        density,
        elasticity: {
          model: elasticity.model,
          youngModulus: elasticity.youngModulus,
          poissonRatio: elasticity.poissonRatio,
          dampingTime: elasticity.dampingTime,
          dampingRatio:
            "dampingRatio" in elasticity ? elasticity.dampingRatio : undefined,
        },
      },
      mount: { min: input.mount.min, max: input.mount.max },
      contact: { min: input.contact.min, max: input.contact.max },
      mesh: input.mesh,
    }),
  );
  const hash = await globalThis.crypto.subtle.digest("SHA-256", encoded);
  const digest = Array.from(new Uint8Array(hash), (byte) =>
    byte.toString(16).padStart(2, "0"),
  ).join("");
  return new CacheEntry({
    version: REDUCER_VERSION,
    dependencies: { input: { value: digest } },
    basePath,
  });
}
