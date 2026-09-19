import {
  RigidBody,
  snapshotGeometry,
  matchesGeometry,
  type GeometrySnapshot,
  type Collider,
  type Joint,
  type Trigger,
} from "@drawcall/physics";
import type { BufferGeometry, Object3D } from "three";
import createCoACD, { type MainModule } from "./coacd.js";
import { heightfield } from "./heightfield.js";

interface Prepared extends GeometrySnapshot {
  shape: { kind: "heightfield" } | { kind: "compound"; hulls: number[][] };
}

/** Only initial collider sources receive prepared geometry; later additions use one hull. */
export class Meshes {
  private readonly owners = new WeakMap<RigidBody, Map<Object3D, Prepared>>();

  async prepare(
    initial: readonly (RigidBody | Joint | Trigger)[],
  ): Promise<void> {
    const shared = new Map<BufferGeometry, Prepared>();
    let api: MainModule | undefined;
    for (const owner of initial) {
      if (!(owner instanceof RigidBody) || owner.disposed) continue;
      const sources = new Map<Object3D, Prepared>();
      for (const collider of owner.getColliders()) {
        const shape = collider.shape();
        if (shape.kind !== "mesh" || shape.approximation !== "trimesh")
          continue;
        let prepared = shared.get(shape.geometry);
        if (!prepared) {
          const data = snapshotGeometry(shape.geometry);
          if (!heightfield(shape.geometry, "grid")) {
            const mesh = manifold(data);
            api ??= await createCoACD();
            const hulls = decompose(api, mesh.positions, mesh.indices);
            if (
              !hulls.length ||
              hulls.some(
                (hull) =>
                  hull.length < 12 ||
                  hull.length % 3 !== 0 ||
                  !hull.every(Number.isFinite),
              )
            )
              throw new Error(
                `CoACD produced invalid hulls for ${owner.name || owner.type}`,
              );
            prepared = { ...data, shape: { kind: "compound", hulls } };
          } else prepared = { ...data, shape: { kind: "heightfield" } };
          shared.set(shape.geometry, prepared);
        }
        sources.set(collider.source, prepared);
      }
      this.owners.set(owner, sources);
    }
  }

  get(owner: RigidBody | Trigger, collider: Collider): Prepared | undefined {
    if (!(owner instanceof RigidBody)) return;
    const prepared = this.owners.get(owner)?.get(collider.source);
    if (!prepared) return;
    const shape = collider.shape();
    if (shape.kind !== "mesh" || shape.approximation !== "trimesh") return;
    return matchesGeometry(prepared, shape.geometry) ? prepared : undefined;
  }
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
    const hulls: number[][] = [];
    for (const hull of result.hulls) {
      try {
        hulls.push(Array.from(hull.vertices));
      } finally {
        hull.vertices.delete();
        hull.indices.delete();
      }
    }
    return hulls;
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
      throw new Error("CoACD requires nondegenerate triangles");
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
      "MuJoCo convex decomposition requires a closed, consistently wound manifold mesh; use an explicit convexHull for open surfaces",
    );
  return {
    positions: new Float64Array(positions),
    indices: new Int32Array(indices),
  };
}
