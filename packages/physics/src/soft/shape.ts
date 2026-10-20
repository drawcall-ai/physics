import { BufferGeometry, Float32BufferAttribute, Vector3 } from "three";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";
import type { Vec3 } from "../colliders/collider.js";
import { validateRange } from "../colliders/geometry.js";
import { tube } from "./tube.js";

export type SoftShape =
  | {
      readonly kind: "volume";
      readonly geometry: BufferGeometry;
      readonly cellSize: number;
    }
  | {
      readonly kind: "volume";
      readonly positions: readonly number[] | Float32Array;
      readonly tetrahedra: readonly number[] | Uint32Array;
    }
  | {
      readonly kind: "surface";
      readonly geometry: BufferGeometry;
      readonly thickness: number;
    }
  | {
      readonly kind: "curve";
      readonly points: readonly Vec3[];
      readonly radius: number;
    };

/** Welded simulation vertices, triangles, and the simulation node for each render vertex. */
export function surfaceMesh(geometry: BufferGeometry): {
  positions: Float32Array;
  triangles: Uint32Array;
  vertices: Uint32Array;
} {
  validateRange(geometry);
  const position = geometry.getAttribute("position");
  if (!position || position.itemSize !== 3 || position.count < 3)
    throw new Error("Soft-body surface needs at least three 3D vertices");
  if (Object.keys(geometry.morphAttributes).length)
    throw new Error("Soft-body rest geometry cannot have morph targets");
  const coordinates: number[] = [];
  let extent = 0;
  for (let i = 0; i < position.count; i++) {
    const x = position.getX(i),
      y = position.getY(i),
      z = position.getZ(i);
    if (![x, y, z].every(Number.isFinite))
      throw new Error("Soft-body vertices must be finite");
    coordinates.push(x, y, z);
    extent = Math.max(extent, Math.abs(x), Math.abs(y), Math.abs(z));
  }
  // Weld position seams independently of UVs and normals, retaining their render mapping.
  const source = new BufferGeometry();
  source.setAttribute("position", new Float32BufferAttribute(coordinates, 3));
  const welded = mergeVertices(source, Math.max(extent * 1e-7, 1e-12));
  const mapping = welded.getIndex();
  if (!mapping) throw new Error("Vertex welding did not produce an index");
  const vertices = Uint32Array.from({ length: mapping.count }, (_, i) =>
    mapping.getX(i),
  );
  const weldedPositions = welded.getAttribute("position");
  const positions = Float32Array.from(
    { length: weldedPositions.count * 3 },
    (_, i) => weldedPositions.getComponent(Math.floor(i / 3), i % 3),
  );
  source.dispose();
  welded.dispose();
  const count = geometry.index?.count ?? position.count;
  if (count % 3)
    throw new Error("Soft-body surface must contain complete triangles");
  const triangles = new Uint32Array(count);
  const a = new Vector3(),
    b = new Vector3(),
    c = new Vector3();
  for (let i = 0; i < count; i++) {
    const index = geometry.index?.getX(i) ?? i;
    const node = vertices[index];
    if (!Number.isInteger(index) || node === undefined)
      throw new Error("Soft-body triangle index is out of range");
    triangles[i] = node;
    if (i % 3 !== 2) continue;
    const i0 = triangles[i - 2],
      i1 = triangles[i - 1];
    if (i0 === undefined || i1 === undefined)
      throw new Error("Incomplete triangle");
    a.fromArray(positions, i0 * 3);
    b.fromArray(positions, i1 * 3);
    c.fromArray(positions, node * 3);
    if (b.sub(a).cross(c.sub(a)).lengthSq() === 0)
      throw new Error("Soft-body surface contains a degenerate triangle");
  }
  return { positions: new Float32Array(positions), triangles, vertices };
}

export function softGeometry(shape: SoftShape): BufferGeometry {
  if (shape.kind === "curve") {
    positive(shape.radius, "Curve radius");
    if (shape.points.length < 2)
      throw new Error("A curve needs at least two points");
    for (const [i, point] of shape.points.entries()) {
      if (point.length !== 3 || !point.every(Number.isFinite))
        throw new Error("Curve points must be finite 3D coordinates");
      const previous = shape.points[i - 1];
      if (previous && point.every((value, j) => value === previous[j]))
        throw new Error("Curve segments must have positive length");
    }
    return tube(
      shape.points.map((p) => new Vector3(...p)),
      shape.radius,
    );
  }
  if ("geometry" in shape) {
    positive(
      shape.kind === "surface" ? shape.thickness : shape.cellSize,
      shape.kind === "surface" ? "Surface thickness" : "Volume cell size",
    );
    const mesh = surfaceMesh(shape.geometry);
    if (shape.kind === "volume") closedSurface(mesh.positions, mesh.triangles);
    return shape.geometry.clone();
  }
  const { positions, tetrahedra } = shape;
  if (
    positions.length < 12 ||
    positions.length % 3 ||
    tetrahedra.length < 4 ||
    tetrahedra.length % 4
  )
    throw new Error("Volume needs 3D vertices and complete tetrahedra");
  if (!Array.from(positions).every(Number.isFinite))
    throw new Error("Volume vertices must be finite");
  const faces = new Map<string, number[]>();
  const counts = new Map<string, number>();
  const cells = new Set<string>();
  const used = new Set<number>();
  const a = new Vector3(),
    b = new Vector3(),
    c = new Vector3(),
    d = new Vector3();
  for (let i = 0; i < tetrahedra.length; i += 4) {
    const ids = Array.from(tetrahedra.slice(i, i + 4));
    const [i0, i1, i2, i3] = ids;
    if (
      i0 === undefined ||
      i1 === undefined ||
      i2 === undefined ||
      i3 === undefined ||
      ids.some(
        (n) => !Number.isInteger(n) || n < 0 || n >= positions.length / 3,
      ) ||
      new Set(ids).size !== 4
    )
      throw new Error("Invalid tetrahedron indices");
    const cell = [...ids].sort((x, y) => x - y).join(",");
    if (cells.has(cell)) throw new Error("Duplicate tetrahedron");
    cells.add(cell);
    ids.forEach((id) => used.add(id));
    a.fromArray(positions, i0 * 3);
    b.fromArray(positions, i1 * 3);
    c.fromArray(positions, i2 * 3);
    d.fromArray(positions, i3 * 3);
    const volume = b.sub(a).cross(c.sub(a)).dot(d.sub(a));
    if (volume <= 0)
      throw new Error("Tetrahedra must have positive signed volume");
    for (const face of [
      [i0, i2, i1],
      [i0, i1, i3],
      [i0, i3, i2],
      [i1, i2, i3],
    ]) {
      const key = [...face].sort((x, y) => x - y).join(",");
      const count = (counts.get(key) ?? 0) + 1;
      if (count > 2) throw new Error("Non-manifold tetrahedral volume");
      counts.set(key, count);
      const previous = faces.get(key);
      if (previous) {
        const start = previous.indexOf(face[0] ?? -1);
        if (previous[(start + 1) % 3] === face[1])
          throw new Error("Tetrahedra overlap across a shared face");
        faces.delete(key);
      } else faces.set(key, face);
    }
  }
  if (used.size !== positions.length / 3)
    throw new Error("Volume contains unused simulation vertices");
  if (!faces.size) throw new Error("Volume has no boundary");
  const geometry = new BufferGeometry();
  geometry.setAttribute("position", new Float32BufferAttribute(positions, 3));
  geometry.setIndex([...faces.values()].flat());
  geometry.computeVertexNormals();
  return geometry;
}

function closedSurface(positions: Float32Array, triangles: Uint32Array): void {
  let volume = 0;
  const origin = new Vector3().fromArray(positions);
  const a = new Vector3(),
    b = new Vector3(),
    c = new Vector3();
  const edges = new Map<string, number>();
  const balance = new Map<string, number>();
  for (let i = 0; i < triangles.length; i += 3) {
    const ids = Array.from(triangles.slice(i, i + 3));
    const [i0, i1, i2] = ids;
    if (i0 === undefined || i1 === undefined || i2 === undefined)
      throw new Error("Incomplete triangle");
    a.fromArray(positions, i0 * 3).sub(origin);
    b.fromArray(positions, i1 * 3).sub(origin);
    c.fromArray(positions, i2 * 3).sub(origin);
    volume += a.dot(b.cross(c));
    for (let j = 0; j < 3; j++) {
      const a = ids[j],
        b = ids[(j + 1) % 3];
      if (a === undefined || b === undefined)
        throw new Error("Incomplete triangle");
      const key = `${Math.min(a, b)},${Math.max(a, b)}`;
      edges.set(key, (edges.get(key) ?? 0) + 1);
      balance.set(key, (balance.get(key) ?? 0) + (a < b ? 1 : -1));
    }
  }
  if ([...edges].some(([key, count]) => count !== 2 || balance.get(key) !== 0))
    throw new Error("Volume surface must be closed and consistently wound");
  if (volume <= 0)
    throw new Error(
      "Volume surface must enclose positive volume with outward winding",
    );
}

function positive(value: number, label: string): void {
  if (!Number.isFinite(value) || value <= 0)
    throw new Error(`${label} must be positive and finite`);
}
