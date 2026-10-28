import { copySoftMaterial } from "@drawcall/physics/backend";
import type { FEMInput, FEMOptions, Bounds } from "./types.js";
import type { Source } from "./source.js";

export function input(source: Source, options: FEMOptions): FEMInput {
  const material = copySoftMaterial(options.material);
  const elastic = material.elasticity;
  if (
    elastic.model !== "linear" &&
    elastic.model !== "neoHookean" &&
    elastic.model !== "saintVenantKirchhoff"
  )
    throw new Error(`FEMLoader does not support ${elastic.model} elasticity`);
  if ("dampingRatio" in elastic && elastic.dampingRatio)
    throw new Error("FEMLoader uses dampingTime, not dampingRatio");
  bounds(options.mount, "mount");
  bounds(options.contact, "contact");
  const minima = [Infinity, Infinity, Infinity];
  const maxima = [-Infinity, -Infinity, -Infinity];
  for (const [i, value] of source.positions.entries()) {
    const axis = i % 3;
    minima[axis] = Math.min(minima[axis] ?? Infinity, value);
    maxima[axis] = Math.max(maxima[axis] ?? -Infinity, value);
  }
  const extent = Math.max(
    ...maxima.map((maximum, i) => maximum - (minima[i] ?? NaN)),
  );
  const mesh = {
    cellSize: options.mesh?.cellSize ?? extent / 8,
    order: options.mesh?.order ?? 2,
    maxDofs: options.mesh?.maxDofs ?? 60,
  };
  if (
    !Number.isFinite(mesh.cellSize) ||
    mesh.cellSize <= 0 ||
    (mesh.order !== 1 && mesh.order !== 2) ||
    !Number.isSafeInteger(mesh.maxDofs) ||
    mesh.maxDofs < 9
  )
    throw new Error("Invalid FEMLoader mesh settings");
  return structuredClone({
    positions: source.positions,
    triangles: source.triangles,
    material,
    mount: options.mount,
    contact: options.contact,
    mesh,
  });
}

function bounds(region: Bounds, label: string): void {
  if (
    !region ||
    !Array.isArray(region.min) ||
    !Array.isArray(region.max) ||
    region.min.length !== 3 ||
    region.max.length !== 3 ||
    ![...region.min, ...region.max].every(Number.isFinite) ||
    region.min.some((minimum, i) => minimum > (region.max[i] ?? -Infinity))
  )
    throw new Error(
      `FEMLoader ${label} must be finite rest-local min/max bounds`,
    );
}
