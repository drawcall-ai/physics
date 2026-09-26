import {
  BoxGeometry,
  CapsuleGeometry,
  CylinderGeometry,
  InstancedMesh,
  Mesh,
  SkinnedMesh,
  SphereGeometry,
  type Vector3,
} from "three";
import type { BufferGeometry, Object3D } from "three";
import type { RigidBody } from "./body.js";
import type { Trigger } from "./trigger.js";
import {
  BoxCollider,
  CapsuleCollider,
  Collider,
  CylinderCollider,
  MeshCollider,
  SphereCollider,
} from "./colliders.js";
import type { Shape } from "./colliders.js";
import { geometryVersion } from "./geometry.js";
import {
  assertPositiveScale,
  assertScaledTransform,
  splitTransform,
} from "./transforms.js";

/** The explicit collider itself, or an automatic one for a mesh, validated for this body. */
export function colliderOf(source: Collider | Mesh, body: RigidBody): Collider {
  for (
    let node: Object3D | null = source;
    node && node !== body;
    node = node.parent
  )
    assertPositiveScale(node, "Collider");
  assertScaledTransform(source.matrixWorld, source.name || source.type);
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

function autoCollider(mesh: Mesh, body: RigidBody): Collider {
  const collider = colliderFromShape(autoShape(mesh, body));
  mesh.matrixWorld.decompose(
    collider.position,
    collider.quaternion,
    collider.scale,
  );
  collider.source = mesh;
  collider.name = mesh.name;
  collider.updateMatrixWorld(true);
  return collider;
}

function colliderFromShape(shape: Shape): Collider {
  switch (shape.kind) {
    case "box":
      return new BoxCollider(shape);
    case "sphere":
      return new SphereCollider(shape);
    case "capsule":
      return new CapsuleCollider(shape);
    case "cylinder":
      return new CylinderCollider(shape);
    case "mesh":
      return new MeshCollider({
        approximation: shape.approximation,
      }).setGeometry(shape.geometry);
  }
}

function unchanged(
  geometry: BufferGeometry,
  reference: BufferGeometry,
): boolean {
  const actual = geometry.getAttribute("position"),
    expected = reference.getAttribute("position");
  if (!actual || actual.count !== expected.count) return false;
  for (let i = 0; i < actual.count; i++) {
    if (
      actual.getX(i) !== expected.getX(i) ||
      actual.getY(i) !== expected.getY(i) ||
      actual.getZ(i) !== expected.getZ(i)
    )
      return false;
  }
  if (geometry.index?.count !== reference.index?.count) return false;
  if (geometry.index && reference.index) {
    for (let i = 0; i < geometry.index.count; i++)
      if (geometry.index.getX(i) !== reference.index.getX(i)) return false;
  }
  if (geometry.drawRange.start !== 0 || geometry.drawRange.count !== Infinity)
    return false;
  return true;
}

/** The shape a visual mesh stands for: an unchanged primitive keeps its shape, other geometry becomes a mesh. */
function autoShape(mesh: Mesh, body: RigidBody): Shape {
  if (
    mesh instanceof SkinnedMesh ||
    mesh instanceof InstancedMesh ||
    Object.keys(mesh.geometry.morphAttributes).length
  )
    throw new Error("Deformed and instanced meshes require explicit colliders");
  const geometry = mesh.geometry;
  validateRange(geometry);
  // The geometry is read again only after it is replaced or marked edited.
  const key = `${geometryVersion(geometry)}/${body.options.colliders}/${body.bodyType}`;
  const known = autoShapes.get(geometry);
  if (known?.key === key) return known.shape;
  const shape = geometryShape(geometry, body);
  autoShapes.set(geometry, { key, shape });
  return shape;
}

const autoShapes = new WeakMap<BufferGeometry, { key: string; shape: Shape }>();

function geometryShape(geometry: BufferGeometry, body: RigidBody): Shape {
  if (body.options.colliders === "box") {
    geometry.computeBoundingBox();
    const bounds = geometry.boundingBox;
    if (!bounds) throw new Error("Mesh has no bounds");
    if (
      Math.abs(bounds.min.x + bounds.max.x) +
        Math.abs(bounds.min.y + bounds.max.y) +
        Math.abs(bounds.min.z + bounds.max.z) >
      1e-6
    )
      throw new Error(
        "Box approximation requires centered geometry; use an explicit collider",
      );
    return {
      kind: "box",
      size: [
        bounds.max.x - bounds.min.x,
        bounds.max.y - bounds.min.y,
        bounds.max.z - bounds.min.z,
      ],
    };
  }
  if (body.options.colliders === "auto") {
    if (
      geometry instanceof BoxGeometry &&
      unchanged(geometry, new BoxGeometry(...boxArgs(geometry)))
    )
      return {
        kind: "box",
        size: [
          geometry.parameters.width,
          geometry.parameters.height,
          geometry.parameters.depth,
        ],
      };
    if (geometry instanceof SphereGeometry) {
      const p = geometry.parameters;
      if (
        p.phiLength === Math.PI * 2 &&
        p.thetaStart === 0 &&
        p.thetaLength === Math.PI &&
        unchanged(
          geometry,
          new SphereGeometry(
            p.radius,
            p.widthSegments,
            p.heightSegments,
            p.phiStart,
            p.phiLength,
            p.thetaStart,
            p.thetaLength,
          ),
        )
      )
        return { kind: "sphere", radius: p.radius };
    }
    if (geometry instanceof CapsuleGeometry) {
      const p = geometry.parameters;
      if (
        unchanged(
          geometry,
          new CapsuleGeometry(
            p.radius,
            p.height,
            p.capSegments,
            p.radialSegments,
            p.heightSegments,
          ),
        )
      )
        return { kind: "capsule", radius: p.radius, height: p.height };
    }
    if (geometry instanceof CylinderGeometry) {
      const p = geometry.parameters;
      if (
        p.radiusTop === p.radiusBottom &&
        !p.openEnded &&
        p.thetaLength === Math.PI * 2 &&
        unchanged(
          geometry,
          new CylinderGeometry(
            p.radiusTop,
            p.radiusBottom,
            p.height,
            p.radialSegments,
            p.heightSegments,
            p.openEnded,
            p.thetaStart,
            p.thetaLength,
          ),
        )
      )
        return { kind: "cylinder", radius: p.radiusTop, height: p.height };
    }
  }
  return {
    kind: "mesh",
    geometry,
    approximation:
      body.options.colliders === "trimesh" ||
      (body.options.colliders === "auto" && body.bodyType === "static")
        ? "trimesh"
        : "convexHull",
  };
}
function boxArgs(
  geometry: BoxGeometry,
): [number, number, number, number, number, number] {
  const p = geometry.parameters;
  return [
    p.width,
    p.height,
    p.depth,
    p.widthSegments,
    p.heightSegments,
    p.depthSegments,
  ];
}
function validateRange(geometry: BufferGeometry): void {
  if (geometry.drawRange.start !== 0 || geometry.drawRange.count !== Infinity) {
    throw new Error("Physics mesh geometry must use the full draw range");
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
