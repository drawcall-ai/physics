import {
  BoxGeometry,
  CapsuleGeometry,
  CylinderGeometry,
  InstancedMesh,
  Mesh,
  SkinnedMesh,
  SphereGeometry,
} from "three";
import type { BufferGeometry, Object3D } from "three";
import type { RigidBody } from "./body.js";
import {
  BoxCollider,
  CapsuleCollider,
  Collider,
  CylinderCollider,
  MeshCollider,
  SphereCollider,
} from "./colliders.js";
import type { Shape } from "./colliders.js";
import { splitTransform } from "./transforms.js";

/** The explicit collider itself, or an automatic one for a mesh, validated for this body. */
export function colliderOf(source: Collider | Mesh, body: RigidBody): Collider {
  for (
    let node: Object3D | null = source;
    node && node !== body;
    node = node.parent
  )
    if (Math.min(node.scale.x, node.scale.y, node.scale.z) <= 0)
      throw new Error(
        `Collider requires positive scale: ${node.name || node.type}`,
      );
  splitTransform(source.matrixWorld, source.name || source.type);
  const collider =
    source instanceof Collider ? source : autoCollider(source, body);
  const shape = collider.shape();
  validateShape(shape);
  if (
    shape.kind === "mesh" &&
    shape.approximation === "trimesh" &&
    body.bodyType !== "static"
  )
    throw new Error("Triangle mesh colliders require static bodies");
  return collider;
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

export function validateShape(shape: Shape): void {
  if (shape.kind === "mesh") {
    validateRange(shape.geometry);
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
