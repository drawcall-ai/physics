import type { BufferGeometry } from "three";

/**
 * The geometry's positions and triangles, welded at render seams and checked for the closed
 * surface CoACD requires without mesh repair.
 */
export function closedMesh(geometry: BufferGeometry): {
  positions: Float64Array;
  indices: Int32Array;
} {
  const position = geometry.getAttribute("position");
  const points = new Map<string, number>();
  const positions: number[] = [],
    remap: number[] = [];
  for (let i = 0; i < position.count; i++) {
    const x = position.getX(i),
      y = position.getY(i),
      z = position.getZ(i);
    const key = `${x},${y},${z}`;
    let index = points.get(key);
    if (index === undefined) {
      index = positions.length / 3;
      points.set(key, index);
      positions.push(x, y, z);
    }
    remap.push(index);
  }
  const triangles = geometry.index;
  const indices: number[] = [];
  for (let i = 0; i < (triangles?.count ?? position.count); i++) {
    const mapped = remap[triangles ? triangles.getX(i) : i];
    if (mapped === undefined) throw new Error("invalid triangle mesh index");
    indices.push(mapped);
  }
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
