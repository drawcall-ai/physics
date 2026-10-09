import type {
  BufferAttribute,
  BufferGeometry,
  InterleavedBufferAttribute,
} from "three";

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
      : "isInterleavedBufferAttribute" in value
        ? `${id(value)}.${id(value.data)}.${value.data.version}`
        : `${id(value)}.${value.version}`;
  return `${id(geometry)}/${attribute(geometry.getAttribute("position"))}/${attribute(geometry.index)}`;
}

export function validateRange(geometry: BufferGeometry): void {
  if (geometry.drawRange.start !== 0 || geometry.drawRange.count !== Infinity) {
    throw new Error("Physics mesh geometry must use the full draw range");
  }
}
