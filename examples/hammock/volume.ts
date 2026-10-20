import { BufferGeometry, Vector3 } from "three";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";

/** Fans this example's convex cargo surfaces into tetrahedra around their center. */
export function volume(surface: BufferGeometry) {
  const source = surface.clone();
  for (const name of Object.keys(source.attributes))
    if (name !== "position") source.deleteAttribute(name);
  const welded = mergeVertices(source);
  const attribute = welded.getAttribute("position");
  const index = welded.getIndex();
  if (!index) throw new Error("Cargo surface needs triangles");
  const center = new Vector3();
  const positions: number[] = [];
  for (let i = 0; i < attribute.count; i++) {
    const point = new Vector3().fromBufferAttribute(attribute, i);
    center.add(point);
    positions.push(...point.toArray());
  }
  center.divideScalar(attribute.count);
  positions.push(...center.toArray());
  const tetrahedra: number[] = [];
  const a = new Vector3(),
    b = new Vector3(),
    c = new Vector3();
  for (let i = 0; i < index.count; i += 3) {
    const i0 = index.getX(i),
      i1 = index.getX(i + 1),
      i2 = index.getX(i + 2);
    a.fromBufferAttribute(attribute, i0).sub(center);
    b.fromBufferAttribute(attribute, i1).sub(center);
    c.fromBufferAttribute(attribute, i2).sub(center);
    if (a.dot(b.cross(c)) <= 0)
      throw new Error(
        "Cargo surface must be convex and outward-wound about its center",
      );
    tetrahedra.push(attribute.count, i0, i1, i2);
  }
  source.dispose();
  welded.dispose();
  return { kind: "volume", positions, tetrahedra } as const;
}
