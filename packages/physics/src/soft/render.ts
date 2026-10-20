import { Vector3 } from "three";
import type { SoftBody } from "./body.js";
import { tube } from "./tube.js";

/** Backends supply world-space render vertices, or centerline nodes for a curve. */
export function updateSoftGeometry(
  body: SoftBody,
  positions: ArrayLike<number>,
): void {
  if (positions.length % 3)
    throw new Error("Incomplete soft-body render positions");
  body.updateWorldMatrix(true, false);
  const inverse = body.matrixWorld.clone().invert();
  const point = new Vector3();
  const { shape } = body.options;
  if (shape.kind === "curve") {
    if (positions.length !== shape.points.length * 3)
      throw new Error("Curve topology changed");
    const points = Array.from({ length: positions.length / 3 }, (_, i) =>
      new Vector3().fromArray(positions, i * 3).applyMatrix4(inverse),
    );
    const geometry = tube(points, shape.radius);
    const target = body.geometry.getAttribute("position"),
      normal = body.geometry.getAttribute("normal");
    const source = geometry.getAttribute("position"),
      sourceNormal = geometry.getAttribute("normal");
    for (let i = 0; i < target.count; i++) {
      target.setXYZ(i, source.getX(i), source.getY(i), source.getZ(i));
      normal.setXYZ(
        i,
        sourceNormal.getX(i),
        sourceNormal.getY(i),
        sourceNormal.getZ(i),
      );
    }
    geometry.dispose();
    target.needsUpdate = true;
    normal.needsUpdate = true;
  } else {
    const attribute = body.geometry.getAttribute("position");
    if (positions.length !== attribute.count * 3)
      throw new Error("Soft-body render topology changed");
    for (let i = 0; i < attribute.count; i++) {
      point.fromArray(positions, i * 3).applyMatrix4(inverse);
      attribute.setXYZ(i, point.x, point.y, point.z);
    }
    attribute.needsUpdate = true;
    body.geometry.computeVertexNormals();
  }
  body.geometry.computeBoundingBox();
  body.geometry.computeBoundingSphere();
}
