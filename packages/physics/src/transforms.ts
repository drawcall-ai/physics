import { Matrix4, Quaternion, Vector3 } from "three";
import type { Collider, Shape } from "./objects.js";
import type { RigidBody } from "./body.js";

export function splitTransform(matrix: Matrix4, name = "Physics transform") {
  const position = new Vector3(),
    rotation = new Quaternion(),
    scale = new Vector3();
  matrix.decompose(position, rotation, scale);
  const composed = new Matrix4().compose(position, rotation, scale);
  if (
    Math.min(scale.x, scale.y, scale.z) <= 0 ||
    !matrix.elements.every(
      (value, index) =>
        Number.isFinite(value) &&
        Math.abs(value - (composed.elements[index] ?? Infinity)) < 1e-6,
    )
  )
    throw new Error(`${name} requires positive scale and no shear`);
  return {
    pose: composed.compose(position, rotation, new Vector3(1, 1, 1)),
    scale,
  };
}

export function resolveCollider(
  body: RigidBody,
  collider: Collider,
  capturedScale?: Vector3,
) {
  const { pose } = splitTransform(body.matrixWorld);
  const transform = pose.invert().multiply(collider.matrixWorld);
  const name = `${body.name || body.type}/${collider.source.name || collider.source.type}`;
  const { pose: matrix, scale } = splitTransform(transform, name);
  return {
    collider,
    matrix,
    scale,
    shape: scaleShape(collider.shape(), capturedScale ?? scale, name),
  };
}

function scaleShape(shape: Shape, scale: Vector3, name: string): Shape {
  const { x, y, z } = scale;
  const radial = Math.abs(x - z) < 1e-6;
  const uniform = radial && Math.abs(x - y) < 1e-6;
  if (
    ((shape.kind === "sphere" || shape.kind === "capsule") && !uniform) ||
    (shape.kind === "cylinder" && !radial)
  )
    throw new Error(
      `Unsupported nonuniform scale for ${shape.kind} at ${name}; use a mesh collider`,
    );
  switch (shape.kind) {
    case "box":
      return {
        kind: "box",
        size: [shape.size[0] * x, shape.size[1] * y, shape.size[2] * z],
      };
    case "mesh":
      return { ...shape, geometry: shape.geometry.clone().scale(x, y, z) };
    case "cylinder":
      return {
        kind: "cylinder",
        radius: shape.radius * x,
        height: shape.height * y,
      };
    case "sphere":
      return { kind: "sphere", radius: shape.radius * x };
    case "capsule":
      return {
        kind: "capsule",
        radius: shape.radius * x,
        length: shape.length * y,
      };
  }
}
