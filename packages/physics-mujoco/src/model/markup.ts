import { Matrix4, Quaternion, Vector3 } from "three";
import { rotation } from "../heap.js";

/** The MuJoCo element name of a physics object. */
export function name(object: { id: number }): string {
  return `o${object.id}`;
}
export function placement(matrix: Matrix4): string {
  return `pos="${new Vector3().setFromMatrixPosition(matrix).toArray().join(" ")}" quat="${rotation(new Quaternion().setFromRotationMatrix(matrix)).join(" ")}"`;
}
