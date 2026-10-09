import {
  BoxGeometry,
  CapsuleGeometry,
  CylinderGeometry,
  InstancedMesh,
  SkinnedMesh,
  SphereGeometry,
  type BufferGeometry,
  type Mesh,
} from "three";
import type { RigidBody } from "../body.js";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
  SphereCollider,
  type Collider,
  type Shape,
} from "./collider.js";
import { geometryVersion, validateRange } from "./geometry.js";

/** A collider standing for a visual mesh, placed where the mesh is. */
export function autoCollider(mesh: Mesh, body: RigidBody): Collider {
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
