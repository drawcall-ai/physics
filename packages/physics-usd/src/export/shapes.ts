import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBodyType, Shape } from "@drawcall/physics";
import { Prim } from "./prim.js";

export function tuple(values: readonly number[]): string {
  return `(${values.join(", ")})`;
}

function transform(
  prim: Prim,
  matrix: Matrix4,
  scale: readonly number[] = [1, 1, 1],
): void {
  const p = new Vector3().setFromMatrixPosition(matrix);
  const q = new Quaternion().setFromRotationMatrix(matrix);
  prim.properties.push(
    `double3 xformOp:translate = ${tuple([p.x, p.y, p.z])}`,
    `quatf xformOp:orient = ${tuple([q.w, q.x, q.y, q.z])}`,
    `double3 xformOp:scale = ${tuple(scale)}`,
    'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]',
  );
}

/** A collider shape as a USD prim; a triangle mesh on a moving body is a convex decomposition. */
export function shapePrim(
  name: string,
  shape: Shape,
  matrix: Matrix4,
  body: RigidBodyType,
): Prim {
  const prim = new Prim(name);
  let scale: readonly number[] = [1, 1, 1];
  switch (shape.kind) {
    case "box":
      prim.type = "Cube";
      prim.properties.push("double size = 1");
      scale = shape.size;
      break;
    case "sphere":
      prim.type = "Sphere";
      prim.properties.push(`double radius = ${shape.radius}`);
      break;
    case "capsule":
      prim.type = "Capsule";
      prim.properties.push(
        `double radius = ${shape.radius}`,
        `double height = ${shape.height}`,
        'uniform token axis = "Y"',
      );
      break;
    case "cylinder":
      prim.type = "Cylinder";
      prim.properties.push(
        `double radius = ${shape.radius}`,
        `double height = ${shape.height}`,
        'uniform token axis = "Y"',
      );
      break;
    case "mesh": {
      prim.type = "Mesh";
      prim.schemas.push("PhysicsMeshCollisionAPI");
      prim.properties.push(
        `uniform token physics:approximation = "${shape.approximation === "convexHull" ? "convexHull" : body === "static" ? "none" : "convexDecomposition"}"`,
      );
      const positions = shape.geometry.getAttribute("position");
      if (!positions)
        throw new Error("Mesh collider has no position attribute");
      const points: string[] = [];
      for (let i = 0; i < positions.count; i++)
        points.push(
          tuple([positions.getX(i), positions.getY(i), positions.getZ(i)]),
        );
      const index = shape.geometry.getIndex();
      const indices = index
        ? Array.from({ length: index.count }, (_, i) => index.getX(i))
        : Array.from({ length: positions.count }, (_, i) => i);
      if (indices.length % 3)
        throw new Error("Mesh collider requires triangle geometry");
      prim.properties.push(
        `point3f[] points = [${points.join(", ")}]`,
        `int[] faceVertexIndices = [${indices.join(", ")}]`,
        `int[] faceVertexCounts = [${Array(indices.length / 3)
          .fill(3)
          .join(", ")}]`,
        'uniform token subdivisionScheme = "none"',
      );
      break;
    }
  }
  transform(prim, matrix, scale);
  return prim;
}
