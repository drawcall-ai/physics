import { Matrix4, Quaternion, Vector3 } from "three";

export type HeapView = Float64Array | Int32Array | Uint8Array;
/** MuJoCo's generated declarations leave heap views untyped. Check that boundary once per access. */
export function array(value: unknown): HeapView {
  if (
    value instanceof Float64Array ||
    value instanceof Int32Array ||
    value instanceof Uint8Array
  )
    return value;
  throw new Error("Expected a MuJoCo numeric heap view");
}
export function at(value: unknown, index: number): number {
  const result = array(value)[index];
  if (result === undefined)
    throw new Error(`MuJoCo array index out of range: ${index}`);
  return result;
}
export function vector(value: unknown, offset: number): Vector3 {
  return new Vector3(
    at(value, offset),
    at(value, offset + 1),
    at(value, offset + 2),
  );
}
export function quaternion(value: unknown, offset: number): Quaternion {
  return new Quaternion(
    at(value, offset + 1),
    at(value, offset + 2),
    at(value, offset + 3),
    at(value, offset),
  );
}
export function rotation(q: Quaternion): number[] {
  return [q.w, q.x, q.y, q.z];
}
export function pose(
  position: unknown,
  rotation: unknown,
  id: number,
): Matrix4 {
  return new Matrix4().compose(
    vector(position, id * 3),
    quaternion(rotation, id * 4),
    new Vector3(1, 1, 1),
  );
}
export function placement(matrix: Matrix4): string {
  return `pos="${new Vector3().setFromMatrixPosition(matrix).toArray().join(" ")}" quat="${rotation(new Quaternion().setFromRotationMatrix(matrix)).join(" ")}"`;
}
export function name(object: { id: number }): string {
  return `o${object.id}`;
}
