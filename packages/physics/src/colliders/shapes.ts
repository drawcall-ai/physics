import type { BufferGeometry, Mesh, Object3D, Vector3 } from "three";
import type { RigidBody } from "../body.js";
import type { Trigger } from "../trigger.js";
import { Collider } from "./collider.js";
import type { Shape } from "./collider.js";
import { autoCollider } from "./auto.js";
import { geometryVersion, validateRange } from "./geometry.js";
import { assertPositiveScale, splitTransform } from "../transforms.js";

/** The explicit collider itself, or an automatic one for a mesh, validated for this body. */
export function colliderOf(source: Collider | Mesh, body: RigidBody): Collider {
  for (
    let node: Object3D | null = source;
    node && node !== body;
    node = node.parent
  )
    assertPositiveScale(node, "Collider");
  splitTransform(source.matrixWorld, source.name || source.type);
  const collider =
    source instanceof Collider ? source : autoCollider(source, body);
  const shape = collider.shape();
  validateShape(shape);
  return collider;
}

/** The collider's pose and scale in its owner's frame, with the shape scaled to world units. */
export function resolveCollider(
  body: RigidBody | Trigger,
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

/** The geometry version each approximation last validated. */
const validated = new WeakMap<BufferGeometry, Map<string, string>>();

export function validateShape(shape: Shape): void {
  if (shape.kind === "mesh") {
    validateRange(shape.geometry);
    // Vertices are checked again only after the geometry is replaced or marked edited.
    const version = geometryVersion(shape.geometry);
    let known = validated.get(shape.geometry);
    if (known?.get(shape.approximation) === version) return;
    const position = shape.geometry.getAttribute("position");
    if (!position || position.count < 3 || position.itemSize !== 3)
      throw new Error("Mesh collider requires position geometry");
    for (let i = 0; i < position.count; i++)
      if (
        ![position.getX(i), position.getY(i), position.getZ(i)].every(
          Number.isFinite,
        )
      )
        throw new Error("Mesh collider positions must be finite");
    if (shape.approximation === "trimesh") {
      const index = shape.geometry.index;
      const count = index?.count ?? position.count;
      if (count % 3 !== 0)
        throw new Error("Triangle mesh requires complete triangles");
      if (index) {
        for (let i = 0; i < index.count; i++) {
          const vertex = index.getX(i);
          if (
            !Number.isInteger(vertex) ||
            vertex < 0 ||
            vertex >= position.count
          )
            throw new Error(
              "Triangle mesh index is outside its position geometry",
            );
        }
      }
    }
    if (!known) validated.set(shape.geometry, (known = new Map()));
    known.set(shape.approximation, version);
    return;
  }
  const dimensions =
    shape.kind === "box"
      ? shape.size
      : shape.kind === "sphere"
        ? [shape.radius]
        : [shape.radius, shape.height];
  if (!dimensions.every((value) => Number.isFinite(value) && value > 0))
    throw new Error("Collider dimensions must be positive");
}
