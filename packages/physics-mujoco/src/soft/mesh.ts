import { type SoftBody } from "@drawcall/physics";
import { surfaceMesh } from "@drawcall/physics/backend";
import { Vector3 } from "three";

export interface SoftMesh {
  body: SoftBody;
  local: Float32Array;
  rest: Float32Array;
  elements: readonly number[];
  vertices: readonly number[];
  dimension: 1 | 2 | 3;
  radius: number;
  thickness: number;
  scale: number;
  /** Lumped volume per node; density is applied when compiling a material. */
  volumes: Float64Array;
}

export function softMesh(body: SoftBody): SoftMesh {
  body.validate();
  if (!("shape" in body.options))
    throw new Error("Reduced FEM bodies use their prepared model");
  const scale = new Vector3().setFromMatrixScale(body.matrixWorld);
  const shape = body.options.shape;
  let local: Float32Array;
  let elements: number[];
  let vertices: number[];
  let dimension: 1 | 2 | 3;
  let radius: number;
  let thickness = 0;
  if (shape.kind === "surface") {
    const mesh = surfaceMesh(shape.geometry);
    local = Float32Array.from(mesh.positions);
    elements = Array.from(mesh.triangles);
    vertices = Array.from(mesh.vertices);
    dimension = 2;
    thickness = shape.thickness * scale.x;
    radius = thickness / 2;
  } else if (shape.kind === "curve") {
    local = Float32Array.from(shape.points.flatMap((point) => [...point]));
    elements = shape.points.slice(1).flatMap((_, i) => [i, i + 1]);
    vertices = shape.points.map((_, i) => i);
    dimension = 1;
    radius = shape.radius * scale.x;
  } else {
    if ("geometry" in shape)
      throw new Error(
        "MuJoCo volume bodies require explicit positions and tetrahedra; automatic volume meshing is not supported",
      );
    local = Float32Array.from(shape.positions);
    elements = Array.from(shape.tetrahedra);
    vertices = Array.from({ length: local.length / 3 }, (_, i) => i);
    dimension = 3;
    radius = 0;
  }
  const rest = new Float32Array(local.length);
  const point = new Vector3();
  for (let i = 0; i < local.length; i += 3)
    point.fromArray(local, i).applyMatrix4(body.matrixWorld).toArray(rest, i);
  const volumes = new Float64Array(local.length / 3);
  for (let i = 0; i < elements.length; i += dimension + 1) {
    const nodes = elements.slice(i, i + dimension + 1);
    const points = nodes.map((node) => new Vector3().fromArray(rest, node * 3));
    const a = points[0],
      b = points[1],
      c = points[2],
      d = points[3];
    if (!a || !b) throw new Error("Incomplete soft element");
    const ab = b.clone().sub(a);
    let volume = ab.length() * Math.PI * radius ** 2;
    if (dimension > 1) {
      if (!c) throw new Error("Incomplete soft surface element");
      const cross = ab.cross(c.clone().sub(a));
      volume = (cross.length() * thickness) / 2;
      if (dimension === 3) {
        if (!d) throw new Error("Incomplete soft volume element");
        volume = Math.abs(cross.dot(d.clone().sub(a))) / 6;
      }
    }
    if (!Number.isFinite(volume) || volume <= 0)
      throw new Error("Soft elements require positive rest volume");
    for (const node of nodes) {
      const current = volumes[node];
      if (current === undefined)
        throw new Error("Soft element references an unknown node");
      volumes[node] = current + volume / nodes.length;
    }
  }
  if (volumes.some((value) => value <= 0))
    throw new Error("Every soft node must belong to a nondegenerate element");
  return {
    body,
    local,
    rest,
    elements,
    vertices,
    dimension,
    radius,
    thickness,
    scale: scale.x,
    volumes,
  };
}
