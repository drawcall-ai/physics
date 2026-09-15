import { BufferGeometry, Vector3 } from "three";
import { ConvexGeometry } from "three/addons/geometries/ConvexGeometry.js";
import type { RigidBody } from "./body.js";
import type { Shape } from "./objects.js";
import { resolveCollider, splitTransform } from "./transforms.js";

/** Infer the authored center without preparing a backend or capturing assembly state. */
export function centerOfMass(body: RigidBody): Vector3 {
  const pose = splitTransform(body.matrixWorld).pose;
  if (body.options.centerOfMass)
    return new Vector3(...body.options.centerOfMass).applyMatrix4(pose);
  const parts = body.getColliders().map((collider) => {
    const { shape, matrix } = resolveCollider(body, collider);
    const { volume, center } = measure(shape);
    return {
      center: center.applyMatrix4(matrix),
      volume,
      density: body.getMaterial(collider).density,
    };
  });
  const massTotal = parts.reduce(
    (total, part) => total + part.volume * part.density,
    0,
  );
  const useVolume = massTotal === 0 && body.options.mass !== undefined;
  const total = useVolume
    ? parts.reduce((sum, part) => sum + part.volume, 0)
    : massTotal;
  const center = new Vector3();
  // An unfinished or zero-density assembly has no inferred center yet.
  if (total === 0) return center.applyMatrix4(pose);
  for (const part of parts)
    center.addScaledVector(
      part.center,
      (part.volume * (useVolume ? 1 : part.density)) / total,
    );
  return center.applyMatrix4(pose);
}

function measure(shape: Shape): { volume: number; center: Vector3 } {
  const center = new Vector3();
  switch (shape.kind) {
    case "box":
      return { volume: shape.size[0] * shape.size[1] * shape.size[2], center };
    case "sphere":
      return { volume: (4 * Math.PI * shape.radius ** 3) / 3, center };
    case "cylinder":
      return { volume: Math.PI * shape.radius ** 2 * shape.height, center };
    case "capsule":
      return {
        volume:
          Math.PI * shape.radius ** 2 * shape.length +
          (4 * Math.PI * shape.radius ** 3) / 3,
        center,
      };
    case "mesh": {
      if (shape.approximation === "trimesh") return measureMesh(shape.geometry);
      const position = shape.geometry.getAttribute("position");
      const points = Array.from({ length: position.count }, (_, index) =>
        new Vector3().fromBufferAttribute(position, index),
      );
      const hull = new ConvexGeometry(points);
      const result = measureMesh(hull);
      hull.dispose();
      return result;
    }
  }
}

function measureMesh(geometry: BufferGeometry): {
  volume: number;
  center: Vector3;
} {
  const position = geometry.getAttribute("position");
  const count = geometry.index?.count ?? position.count;
  const origin = new Vector3().fromBufferAttribute(position, 0);
  const vertex = (index: number) =>
    new Vector3()
      .fromBufferAttribute(position, geometry.index?.getX(index) ?? index)
      .sub(origin);
  const center = new Vector3();
  let volume = 0;
  for (let index = 0; index < count; index += 3) {
    const a = vertex(index),
      b = vertex(index + 1),
      c = vertex(index + 2);
    // Each oriented face and the reference point bound a signed tetrahedron.
    const weight = a.dot(b.clone().cross(c)) / 6;
    volume += weight;
    center.addScaledVector(a.add(b).add(c), weight / 4);
  }
  if (volume !== 0) center.divideScalar(volume).add(origin);
  return { volume: Math.abs(volume), center };
}
