import type * as Rapier from "@dimforge/rapier3d-compat";
import type { Collider, RigidBody, Shape } from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";

type API = typeof Rapier;

export function collider(
  api: API,
  source: Collider,
  body: RigidBody,
): Rapier.ColliderDesc {
  const shape = source.shape();
  const material = body.getMaterial(source);
  const matrix = body.matrixWorld.clone().invert().multiply(source.matrixWorld);
  const position = new Vector3().setFromMatrixPosition(matrix);
  const quaternion = new Quaternion().setFromRotationMatrix(matrix);
  if (material.staticFriction !== material.dynamicFriction)
    throw new Error("Rapier requires equal static and dynamic friction.");
  const result = descriptor(api, shape);
  if (source.collisionGroups) {
    const { membership, filter } = source.collisionGroups;
    result.setCollisionGroups(((membership << 16) | filter) >>> 0);
  }
  return result
    .setTranslation(position.x, position.y, position.z)
    .setRotation(quaternion)
    .setDensity(material.density)
    .setFriction(material.dynamicFriction)
    .setRestitution(material.restitution)
    .setSensor(source.sensor);
}

function descriptor(api: API, shape: Shape): Rapier.ColliderDesc {
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
      return factory.capsule(shape.length / 2, shape.radius);
    case "cylinder":
      return factory.cylinder(shape.height / 2, shape.radius);
    case "mesh": {
      const positions = shape.geometry.getAttribute("position");
      if (!positions) throw new Error("Mesh collider needs positions.");
      const vertices = new Float32Array(positions.count * 3);
      for (let i = 0; i < positions.count; i++)
        vertices.set(
          [positions.getX(i), positions.getY(i), positions.getZ(i)],
          i * 3,
        );
      if (shape.approximation === "convexHull") {
        const hull = factory.convexHull(vertices);
        if (!hull)
          throw new Error("Rapier could not construct the convex hull.");
        return hull;
      }
      const index = shape.geometry.getIndex();
      const indices = new Uint32Array(index ? index.count : positions.count);
      for (let i = 0; i < indices.length; i++)
        indices[i] = index ? index.getX(i) : i;
      return factory.trimesh(vertices, indices);
    }
  }
}
