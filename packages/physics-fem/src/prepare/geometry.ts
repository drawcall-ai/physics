import { MeshoptSimplifier } from "meshoptimizer";
import { Box3, BufferAttribute, BufferGeometry, Vector3 } from "three";
import { MeshBVH } from "three-mesh-bvh";

export { tetrahedralize } from "./mesh.js";

export async function initialize() {
  await MeshoptSimplifier.ready;
}

export function closest(
  positions: ArrayLike<number>,
  triangles: ArrayLike<number>,
  queries: ArrayLike<number>,
) {
  if (triangles.length === 0)
    throw new Error("Cannot project onto an empty surface");
  const geometry = new BufferGeometry();
  geometry.setAttribute(
    "position",
    new BufferAttribute(new Float64Array(positions), 3),
  );
  geometry.setIndex(new BufferAttribute(new Uint32Array(triangles), 1));
  // Contact faces refer to the caller's triangle order; BVH construction must preserve it.
  const bvh = new MeshBVH(geometry, { indirect: true });
  const points = new Float64Array(queries.length);
  const distances = new Float64Array(queries.length / 3);
  const faces = new Uint32Array(queries.length / 3);
  const query = new Vector3();
  const target = { point: new Vector3(), distance: 0, faceIndex: 0 };
  try {
    for (let i = 0; i < distances.length; i++) {
      query.fromArray(queries, i * 3);
      const hit = bvh.closestPointToPoint(query, target);
      if (!hit) throw new Error("Surface projection failed");
      hit.point.toArray(points, i * 3);
      distances[i] = hit.distance;
      faces[i] = hit.faceIndex;
    }
    return { points, distances, faces };
  } finally {
    geometry.dispose();
  }
}

export function simplify(
  positions: ArrayLike<number>,
  triangles: ArrayLike<number>,
  targetFaces: number,
) {
  const source = new Float64Array(positions);
  const indices = new Uint32Array(triangles);
  const bounds = new Box3().setFromBufferAttribute(
    new BufferAttribute(source, 3),
  );
  const center = bounds.getCenter(new Vector3());
  const extent = bounds.getSize(new Vector3());
  const size = Math.max(extent.x, extent.y, extent.z);
  if (!Number.isFinite(size) || size <= 0)
    throw new Error("Contact surface must have positive extent");
  const normalized = Float32Array.from(
    source,
    (value, i) => (value - center.getComponent(i % 3)) / size,
  );
  const [simplified] = MeshoptSimplifier.simplify(
    indices,
    normalized,
    3,
    targetFaces * 3,
    1,
    ["LockBorder"],
  );
  const [remap, count] = MeshoptSimplifier.compactMesh(simplified);
  const compact = new Float64Array(count * 3);
  remap.forEach((next, previous) => {
    if (next < count)
      compact.set(source.subarray(previous * 3, previous * 3 + 3), next * 3);
  });
  return { positions: compact, triangles: simplified };
}
