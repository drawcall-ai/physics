import { CacheEntry } from "vitecache";
import type { FEMInput } from "./types.js";
import { REDUCER_VERSION } from "./types.js";

/**
 * Keyed by the inputs that shape the reduction: the geometry the object function creates rather than its source, while
 * contact friction and restitution stay adjustable without rebaking.
 */
export function cache(input: FEMInput): CacheEntry {
  const { density, elasticity } = input.material;
  return new CacheEntry({
    version: REDUCER_VERSION,
    dependencies: {
      input: {
        value: {
          positions: input.positions,
          triangles: input.triangles,
          material: {
            density,
            elasticity: {
              model: elasticity.model,
              youngModulus: elasticity.youngModulus,
              poissonRatio: elasticity.poissonRatio,
              dampingTime: elasticity.dampingTime ?? null,
            },
          },
          mount: { min: input.mount.min, max: input.mount.max },
          contact: { min: input.contact.min, max: input.contact.max },
          mesh: input.mesh,
        },
      },
    },
  });
}
