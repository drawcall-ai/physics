import type { BufferGeometry } from "three";

export interface GeometrySnapshot {
  positions: number[];
  indices: number[];
}

/** Collision data, independent of render attributes and their buffer layout. */
export function snapshotGeometry(geometry: BufferGeometry): GeometrySnapshot {
  const position = geometry.getAttribute("position");
  const positions: number[] = [];
  for (let i = 0; i < position.count; i++)
    positions.push(position.getX(i), position.getY(i), position.getZ(i));
  const index = geometry.index;
  const indices = Array.from(
    { length: index?.count ?? position.count },
    (_, i) => (index ? index.getX(i) : i),
  );
  return { positions, indices };
}

/** Compare current collision data without allocating another snapshot. */
export function matchesGeometry(
  snapshot: GeometrySnapshot,
  geometry: BufferGeometry,
): boolean {
  const position = geometry.getAttribute("position");
  const index = geometry.index;
  if (
    snapshot.positions.length !== position.count * 3 ||
    snapshot.indices.length !== (index?.count ?? position.count)
  )
    return false;
  for (let i = 0; i < position.count; i++)
    if (
      snapshot.positions[i * 3] !== position.getX(i) ||
      snapshot.positions[i * 3 + 1] !== position.getY(i) ||
      snapshot.positions[i * 3 + 2] !== position.getZ(i)
    )
      return false;
  return snapshot.indices.every(
    (value, i) => value === (index ? index.getX(i) : i),
  );
}
