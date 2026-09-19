import { BufferGeometry, Float32BufferAttribute } from "three";
const vertices = new Float64Array([
  -0.48, -0.58, -0.25, 0.72, -0.58, -0.25, 0.72, 0.02, -0.25, 0.12, 0.02, -0.25,
  0.12, 0.82, -0.25, -0.48, 0.82, -0.25, -0.48, -0.58, 0.25, 0.72, -0.58, 0.25,
  0.72, 0.02, 0.25, 0.12, 0.02, 0.25, 0.12, 0.82, 0.25, -0.48, 0.82, 0.25,
]);
// prettier-ignore
const faces = new Int32Array([
  2, 1, 0,   5, 4, 3,   3, 2, 0,   0, 5, 3,   6, 7, 8,
  9, 10, 11, 6, 8, 9,   9, 11, 6,  7, 6, 1,   1, 6, 0,
  8, 7, 2,   2, 7, 1,   9, 8, 3,   3, 8, 2,   10, 9, 4,
  4, 9, 3,   6, 11, 0,  0, 11, 5,  11, 10, 5, 5, 10, 4,
]);

export function concave() {
  return new BufferGeometry()
    .setAttribute("position", new Float32BufferAttribute(vertices, 3))
    .setIndex([...faces]);
}
