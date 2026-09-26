import type { BufferGeometry, Vector3 } from "three";
import { RigidBody } from "./body.js";
import type { Collider } from "./colliders.js";
import {
  matchesGeometry,
  snapshotGeometry,
  type GeometrySnapshot,
} from "./geometry.js";
import type { MainModule } from "./coacd.js";

/**
 * Convex parts of closed triangle meshes, for backends that cannot collide a concave mesh:
 * every triangle mesh in MuJoCo, and those on moving bodies in Rapier. CoACD decomposes each
 * mesh once, when a world is built, and loads only if some mesh needs it; backends read the
 * parts back while creating colliders.
 */
const prepared = new WeakMap<
  BufferGeometry,
  GeometrySnapshot & { parts: number[][] }
>();
let coacd: Promise<MainModule> | undefined;

/** Decomposes the triangle mesh colliders of the initial bodies that `needs` selects. */
export async function prepareConvexParts(
  initial: readonly unknown[],
  needs: (body: RigidBody, geometry: BufferGeometry) => boolean,
): Promise<void> {
  for (const body of initial) {
    if (!(body instanceof RigidBody) || body.disposed) continue;
    for (const collider of body.getColliders()) {
      const shape = collider.shape();
      if (shape.kind !== "mesh" || shape.approximation !== "trimesh") continue;
      const { geometry } = shape;
      if (current(geometry) || !needs(body, geometry)) continue;
      try {
        const data = snapshotGeometry(geometry);
        const { positions, indices } = manifold(data);
        coacd ??= import("./coacd.js")
          .then((module) => module.default())
          .catch((error: unknown) => {
            coacd = undefined;
            throw error;
          });
        const parts = decompose(await coacd, positions, indices);
        if (
          !parts.length ||
          parts.some(
            (part) =>
              part.length < 12 ||
              part.length % 3 !== 0 ||
              !part.every(Number.isFinite),
          )
        )
          throw new Error("CoACD produced invalid convex parts");
        prepared.set(geometry, { ...data, parts });
      } catch (error) {
        const name = `${body.name || body.type}/${collider.source.name || collider.source.type}`;
        throw new Error(
          `Convex decomposition failed for ${name}: ${error instanceof Error ? error.message : String(error)}`,
          { cause: error },
        );
      }
    }
  }
}

/**
 * A triangle mesh collider's prepared convex parts at `scale`, as flat vertex lists; none if
 * it was not decomposed when the world was built or its mesh changed since.
 */
export function convexParts(
  collider: Collider,
  scale: Vector3,
): number[][] | undefined {
  const shape = collider.shape();
  if (shape.kind !== "mesh" || shape.approximation !== "trimesh") return;
  return current(shape.geometry)?.parts.map((part) =>
    part.map((value, i) => value * scale.getComponent(i % 3)),
  );
}

function current(geometry: BufferGeometry) {
  const entry = prepared.get(geometry);
  return entry && matchesGeometry(entry, geometry) ? entry : undefined;
}

function decompose(
  api: MainModule,
  positions: Float64Array,
  indices: Int32Array,
): number[][] {
  const result = api.decompose(
    positions,
    indices,
    0.05,
    -1,
    50,
    2000,
    20,
    150,
    3,
    256,
    true,
  );
  try {
    const parts: number[][] = [];
    for (const hull of result.hulls) {
      try {
        parts.push(Array.from(hull.vertices));
      } finally {
        hull.vertices.delete();
        hull.indices.delete();
      }
    }
    return parts;
  } finally {
    result.hulls.delete();
  }
}

/** Weld render seams before checking the closed surface required by CoACD without mesh repair. */
function manifold(mesh: GeometrySnapshot) {
  const points = new Map<string, number>();
  const positions: number[] = [],
    remap: number[] = [];
  for (let i = 0; i < mesh.positions.length; i += 3) {
    const point = mesh.positions.slice(i, i + 3);
    const key = point.join(",");
    let index = points.get(key);
    if (index === undefined) {
      index = positions.length / 3;
      points.set(key, index);
      positions.push(...point);
    }
    remap.push(index);
  }
  const indices = mesh.indices.map((index) => {
    const mapped = remap[index];
    if (mapped === undefined) throw new Error("Invalid triangle mesh index");
    return mapped;
  });
  const edges = new Map<string, { count: number; direction: number }>();
  for (let i = 0; i < indices.length; i += 3) {
    const a = indices[i],
      b = indices[i + 1],
      c = indices[i + 2];
    if (
      a === undefined ||
      b === undefined ||
      c === undefined ||
      a === b ||
      b === c ||
      c === a
    )
      throw new Error("the mesh has degenerate triangles");
    for (const [from, to] of [
      [a, b],
      [b, c],
      [c, a],
    ] as const) {
      const key = `${Math.min(from, to)},${Math.max(from, to)}`;
      const edge = edges.get(key) ?? { count: 0, direction: 0 };
      edge.count++;
      edge.direction += from < to ? 1 : -1;
      edges.set(key, edge);
    }
  }
  if (
    [...edges.values()].some((edge) => edge.count !== 2 || edge.direction !== 0)
  )
    throw new Error(
      "the mesh is not closed and consistently wound; use an explicit convexHull for open surfaces",
    );
  return {
    positions: new Float64Array(positions),
    indices: new Int32Array(indices),
  };
}
