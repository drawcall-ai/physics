import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  convexParts,
  resolveCollider,
  resolveCollisionGroups,
} from "@drawcall/physics";
import type { RigidBody, Shape } from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";

/**
 * Rapier colliders for one authored collider. Rapier keeps real triangle meshes for static
 * bodies; on a moving body a triangle mesh collides as the convex parts prepared when the
 * world was built, or as one hull without them.
 */
export function colliderDescs(
  api: typeof Rapier,
  resolved: ReturnType<typeof resolveCollider>,
  body: RigidBody,
): Rapier.ColliderDesc[] {
  const { collider, shape, matrix, scale } = resolved;
  const material = body.getMaterial(collider);
  const position = new Vector3().setFromMatrixPosition(matrix);
  const quaternion = new Quaternion().setFromRotationMatrix(matrix);
  if (material.staticFriction !== material.dynamicFriction)
    throw new Error("Rapier requires equal static and dynamic friction");
  const { membership, filter } = resolveCollisionGroups(collider, body);
  const moving =
    shape.kind === "mesh" &&
    shape.approximation === "trimesh" &&
    body.bodyType !== "static";
  const parts = moving ? convexParts(collider, scale) : undefined;
  const descriptors = parts
    ? parts.map((part) => hull(api, new Float32Array(part)))
    : [
        descriptor(
          api,
          moving ? { ...shape, approximation: "convexHull" } : shape,
        ),
      ];
  return descriptors.map((result) =>
    result
      .setCollisionGroups(((membership << 16) | filter) >>> 0)
      .setTranslation(position.x, position.y, position.z)
      .setRotation(quaternion)
      .setDensity(material.density)
      .setFriction(material.dynamicFriction)
      .setRestitution(material.restitution),
  );
}

function hull(api: typeof Rapier, vertices: Float32Array): Rapier.ColliderDesc {
  const result = api.ColliderDesc.convexHull(vertices);
  if (!result) throw new Error("Rapier could not construct the convex hull");
  return result;
}

export function descriptor(
  api: typeof Rapier,
  shape: Shape,
): Rapier.ColliderDesc {
  const factory = api.ColliderDesc;
  switch (shape.kind) {
    case "box":
      return factory.cuboid(
        shape.size[0] / 2,
        shape.size[1] / 2,
        shape.size[2] / 2,
      );
    case "sphere":
      return factory.ball(shape.radius);
    case "capsule":
      return factory.capsule(shape.height / 2, shape.radius);
    case "cylinder":
      return factory.cylinder(shape.height / 2, shape.radius);
    case "mesh": {
      const positions = shape.geometry.getAttribute("position");
      if (!positions) throw new Error("Mesh collider needs positions");
      const vertices = new Float32Array(positions.count * 3);
      for (let i = 0; i < positions.count; i++)
        vertices.set(
          [positions.getX(i), positions.getY(i), positions.getZ(i)],
          i * 3,
        );
      if (shape.approximation === "convexHull") return hull(api, vertices);
      const index = shape.geometry.getIndex();
      const indices = new Uint32Array(index ? index.count : positions.count);
      for (let i = 0; i < indices.length; i++)
        indices[i] = index ? index.getX(i) : i;
      return factory.trimesh(vertices, indices);
    }
  }
}
