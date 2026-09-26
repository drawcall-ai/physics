import {
  InterleavedBufferAttribute,
  type BufferAttribute,
  type BufferGeometry,
} from "three";

export interface GeometrySnapshot {
  positions: number[];
  indices: number[];
  version: string;
}

const ids = new WeakMap<object, number>();
let next = 0;
function id(object: object): number {
  let value = ids.get(object);
  if (value === undefined) ids.set(object, (value = ++next));
  return value;
}

/**
 * Changes when a geometry's positions or triangles are replaced or marked edited, as three.js
 * requires for any edit (`needsUpdate`), so change detection never reads the vertices.
 */
export function geometryVersion(geometry: BufferGeometry): string {
  const attribute = (
    value: BufferAttribute | InterleavedBufferAttribute | null,
  ) =>
    !value
      ? "-"
      : value instanceof InterleavedBufferAttribute
        ? `${id(value)}.${id(value.data)}.${value.data.version}`
        : `${id(value)}.${value.version}`;
  return `${id(geometry)}/${attribute(geometry.getAttribute("position"))}/${attribute(geometry.index)}`;
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
  return { positions, indices, version: geometryVersion(geometry) };
}

/** Whether a geometry is still as it was when snapshotted. */
export function matchesGeometry(
  snapshot: GeometrySnapshot,
  geometry: BufferGeometry,
): boolean {
  return snapshot.version === geometryVersion(geometry);
}
