import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  resolveCollider,
  type Collider,
  type RigidBody,
  type Shape,
  type Trigger,
} from "@drawcall/physics";
import {
  convexParts,
  geometryVersion,
  resolveCollisionGroups,
} from "@drawcall/physics/backend";
import { Quaternion, Vector3, type Matrix4, type Object3D } from "three";

type Resolved = ReturnType<typeof resolveCollider>;

/**
 * Rapier colliders for one authored body collider. Rapier keeps real triangle meshes for static
 * bodies; on a moving body a triangle mesh collides as the convex parts prepared when the world
 * was built, or as one hull without them.
 */
export function colliderDescs(
  api: typeof Rapier,
  resolved: Resolved,
  body: RigidBody,
): Rapier.ColliderDesc[] {
  const { collider, shape, matrix, scale } = resolved;
  const material = body.getMaterial(collider);
  if (material.staticFriction !== material.dynamicFriction)
    throw new Error("Rapier requires equal static and dynamic friction");
  const moving =
    shape.kind === "mesh" &&
    shape.approximation === "trimesh" &&
    body.bodyType !== "static";
  const parts = moving ? convexParts(collider, scale) : undefined;
  const descs = parts
    ? parts.map((part) => hull(api, new Float32Array(part)))
    : [
        descriptor(
          api,
          moving ? { ...shape, approximation: "convexHull" } : shape,
        ),
      ];
  return descs.map((desc) =>
    place(desc, matrix, collider, body)
      .setDensity(material.density)
      .setFriction(material.dynamicFriction)
      .setRestitution(material.restitution),
  );
}

/** Places the desc at `matrix` in its Rapier body's frame, in the collider's groups. */
export function place(
  desc: Rapier.ColliderDesc,
  matrix: Matrix4,
  collider: Collider,
  owner: RigidBody | Trigger,
): Rapier.ColliderDesc {
  const { membership, filter } = resolveCollisionGroups(collider, owner);
  const position = new Vector3().setFromMatrixPosition(matrix);
  return desc
    .setTranslation(position.x, position.y, position.z)
    .setRotation(new Quaternion().setFromRotationMatrix(matrix))
    .setCollisionGroups(((membership << 16) | filter) >>> 0);
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

function hull(api: typeof Rapier, vertices: Float32Array): Rapier.ColliderDesc {
  const desc = api.ColliderDesc.convexHull(vertices);
  if (!desc) throw new Error("Rapier could not construct the convex hull");
  return desc;
}

/**
 * Resolves the collider at the scale captured when it first joined, which the returned scale
 * reports; Rapier shapes cannot follow a later scale edit.
 */
export function resolve(
  owner: RigidBody | Trigger,
  collider: Collider,
  scales: ReadonlyMap<Object3D, Vector3>,
): Resolved {
  const captured = scales.get(collider.source);
  const resolved = resolveCollider(owner, collider, captured);
  const name = `${owner.name}/${collider.name || collider.type}`;
  assertScale("Collider", name, captured, resolved.scale);
  return { ...resolved, scale: captured ?? resolved.scale };
}

export function assertScale(
  kind: "Body" | "Collider",
  name: string,
  captured: Vector3 | undefined,
  scale: Vector3,
): void {
  if (captured && captured.distanceTo(scale) > 1e-6)
    throw new Error(
      `${kind} scale cannot change after backend initialization: ${name} (${captured.toArray()} → ${scale.toArray()}); recreate it`,
    );
}

/** What decides the collider's Rapier form, relative to `owner`. */
export function fingerprint(collider: Collider, owner: Object3D): unknown {
  const shape = collider.shape();
  const data =
    shape.kind === "mesh"
      ? {
          kind: shape.kind,
          approximation: shape.approximation,
          version: geometryVersion(shape.geometry),
        }
      : shape;
  // Relative transforms accumulate tiny roundoff as bodies move under parents.
  const transform = owner.matrixWorld
    .clone()
    .invert()
    .multiply(collider.matrixWorld)
    .elements.map((value) => Math.round(value * 1e10) / 1e10);
  return [collider.source.uuid, data, transform, collider.version];
}

/** Removes the colliders behind `sources`; those of a removed body went with it. */
export function removeColliders(
  native: Rapier.World,
  sources: Map<number, Object3D>,
): void {
  for (const handle of sources.keys()) {
    const collider = native.getCollider(handle);
    if (collider) native.removeCollider(collider, true);
  }
  sources.clear();
}
