import { Matrix4, Quaternion, Vector3 } from "three";
import type { Collider, Shape } from "./colliders.js";
import type { RigidBody } from "./body.js";

export function validateVector(value: Vector3): void {
  if (![value.x, value.y, value.z].every(Number.isFinite))
    throw new Error("Physics vectors must be finite");
}

/** Rejects matrices a body or joint frame cannot carry: any scale or shear. */
export function assertRigidTransform(matrix: Matrix4): void {
  const { scale } = splitTransform(matrix);
  if ([scale.x, scale.y, scale.z].some((value) => Math.abs(value - 1) > 1e-6))
    throw new Error("Physics transforms must have unit scale and no shear");
}

/** The unit vector an axis token names. */
export function axisVector(axis: "X" | "Y" | "Z"): Vector3 {
  if (axis === "X") return new Vector3(1, 0, 0);
  if (axis === "Y") return new Vector3(0, 1, 0);
  return new Vector3(0, 0, 1);
}

/** Writes a world pose into the object's local transform, keeping its world scale. */
export function setWorldPose(object: RigidBody, pose: Matrix4): void {
  object.updateWorldMatrix(true, false);
  const scale = splitTransform(object.matrixWorld).scale;
  const matrix = pose.clone().scale(scale);
  if (object.parent)
    matrix.premultiply(object.parent.matrixWorld.clone().invert());
  splitTransform(matrix);
  matrix.decompose(object.position, object.quaternion, object.scale);
  object.updateMatrix();
  object.updateMatrixWorld(true);
}

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
        height: shape.height * y,
      };
  }
}
