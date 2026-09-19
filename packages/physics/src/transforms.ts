import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
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

/** Rejects matrices that do not split into a pose and a positive scale: shear, mirroring, or non-finite terms. */
export function assertScaledTransform(matrix: Matrix4, name?: string): void {
  splitTransform(matrix, name);
}

export function assertPositiveScale(node: Object3D, subject: string): void {
  if (Math.min(node.scale.x, node.scale.y, node.scale.z) <= 0)
    throw new Error(
      `${subject} requires positive scale: ${node.name || node.type}`,
    );
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
  assertScaledTransform(matrix);
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
