import { BoxGeometry, Mesh, Vector3 } from "three";
import { beforeAll, expect, it } from "vitest";
import {
  closest,
  initialize,
  simplify,
  tetrahedralize,
} from "../src/prepare/geometry.js";
import { source } from "../src/source.js";

beforeAll(initialize);

it("preserves double-precision projections and original triangle identities", () => {
  const positions: number[] = [];
  const triangles: number[] = [];
  for (let i = 0; i < 40; i++) {
    const x = 1e6 + 0.123456789 + (39 - i) * 2;
    positions.push(x, 0, 0, x + 1, 0, 0, x, 1, 0);
    triangles.push(i * 3, i * 3 + 1, i * 3 + 2);
  }
  const x = 1e6 + 0.123456789 + 8;
  const result = closest(positions, triangles, [x + 0.2, 0.3, 0.7]);
  expect(result.faces[0]).toBe(35);
  expect(result.points[0]).toBeCloseTo(x + 0.2, 9);
  expect(result.points[1]).toBeCloseTo(0.3, 12);
  expect(result.distances[0]).toBeCloseTo(0.7, 12);
});

it.each([0, 1e6])(
  "simplifies contact interiors at offset %s while preserving their borders and positions",
  (offset) => {
    const positions: number[] = [];
    const triangles: number[] = [];
    const border: number[][] = [];
    for (let y = 0; y < 5; y++) {
      for (let x = 0; x < 5; x++) {
        const point = [offset + x / 7 + 1e-10, offset + y / 7 + 2e-10, 0];
        positions.push(...point);
        if (x === 0 || y === 0 || x === 4 || y === 4) border.push(point);
        if (x < 4 && y < 4) {
          const i = y * 5 + x;
          triangles.push(i, i + 1, i + 5, i + 1, i + 6, i + 5);
        }
      }
    }
    const result = simplify(positions, triangles, 20);
    expect(result.triangles.length).toBeLessThan(triangles.length);
    const points = Array.from({ length: result.positions.length / 3 }, (_, i) =>
      Array.from(result.positions.subarray(i * 3, i * 3 + 3)),
    );
    for (const point of border) expect(points).toContainEqual(point);
    for (const index of result.triangles)
      expect(index).toBeLessThan(points.length);
  },
);

it("meshes closed solids with positive tetrahedra at different physical scales", async () => {
  const origin = new Vector3(),
    a = new Vector3(),
    b = new Vector3(),
    c = new Vector3();
  for (const scale of [1, 0.001]) {
    const mesh = new Mesh(new BoxGeometry(scale, scale, scale));
    const surface = source(mesh);
    const side =
      Math.max(...surface.positions) - Math.min(...surface.positions);
    const result = await tetrahedralize(
      surface.positions,
      surface.triangles,
      scale / 3,
    );
    expect(result.tetrahedra.length).toBeGreaterThan(0);
    let volume = 0;
    for (let i = 0; i < result.tetrahedra.length; i += 4) {
      const [ia, ib, ic, id] = result.tetrahedra.subarray(i, i + 4);
      if (
        ia === undefined ||
        ib === undefined ||
        ic === undefined ||
        id === undefined
      )
        throw new Error("Incomplete cell");
      origin.fromArray(result.positions, ia * 3);
      a.fromArray(result.positions, ib * 3).sub(origin);
      b.fromArray(result.positions, ic * 3).sub(origin);
      c.fromArray(result.positions, id * 3).sub(origin);
      const determinant = a.dot(b.cross(c));
      expect(determinant).toBeGreaterThan(0);
      volume += determinant / 6;
    }
    expect(volume / side ** 3).toBeCloseTo(1, 9);
    mesh.geometry.dispose();
  }
}, 30_000);
