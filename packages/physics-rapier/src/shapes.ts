import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  resolveCollider,
  type Collider,
  type Shape,
  type Trigger,
} from "@drawcall/physics";
import {
  convexParts,
  geometryVersion,
  lockScale,
  partsVersion,
  resolveCollisionGroups,
  vertices,
  type Decomposes,
} from "@drawcall/physics/backend";
import { Quaternion, Vector3, type Matrix4, type Object3D } from "three";

type Resolved = ReturnType<typeof resolveCollider>;

/** Rapier keeps real triangle meshes for static bodies; on moving ones they collide as convex parts. */
export const decomposes: Decomposes = (body) => body.bodyType !== "static";

/** Whether the shape collides as convex parts on `owner`. */
function decomposed(shape: Shape, owner: Object3D): boolean {
  return (
    shape.kind === "mesh" &&
    shape.approximation === "trimesh" &&
    owner instanceof RigidBody &&
    decomposes(owner, shape.geometry)
  );
}

/** Rapier colliders for one authored body collider. */
export function colliderDescs(
  api: typeof Rapier,
  resolved: Resolved,
  body: RigidBody,
): Rapier.ColliderDesc[] {
  const { collider, shape, matrix, scale } = resolved;
  const material = body.getMaterial(collider);
  if (material.staticFriction !== material.dynamicFriction)
    throw new Error("Rapier requires equal static and dynamic friction");
  if (material.torsionalFriction || material.rollingFriction)
    throw new Error("Rapier colliders have no torsional or rolling friction");
  const descs = decomposed(shape, body)
    ? convexParts(body, collider, scale).map((part) =>
        hull(api, new Float32Array(part)),
      )
    : [descriptor(api, shape)];
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
      const points = new Float32Array(vertices(shape.geometry));
      if (shape.approximation === "convexHull") return hull(api, points);
      const index = shape.geometry.getIndex();
      const indices = new Uint32Array(index ? index.count : points.length / 3);
      for (let i = 0; i < indices.length; i++)
        indices[i] = index ? index.getX(i) : i;
      return factory.trimesh(points, indices);
    }
  }
}

function hull(api: typeof Rapier, points: Float32Array): Rapier.ColliderDesc {
  const desc = api.ColliderDesc.convexHull(points);
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
  return { ...resolved, scale: lockScale(name, captured, resolved.scale) };
}

/** What decides the collider's Rapier form, relative to `owner`. */
export function fingerprint(collider: Collider, owner: Object3D): unknown {
  const shape = collider.shape();
  const data =
    shape.kind === "mesh"
      ? {
          kind: shape.kind,
          approximation: shape.approximation,
          // Only colliders made of convex parts change once those arrive.
          version: decomposed(shape, owner)
            ? partsVersion(shape.geometry)
            : geometryVersion(shape.geometry),
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

/**
 * Removes the colliders behind `sources`. Those of a removed body went with it, and Rapier may
 * already have reused their handles, so only colliders that are still valid are removed.
 */
export function removeColliders(
  simulation: Rapier.World,
  sources: Map<Rapier.Collider, Object3D>,
): void {
  for (const collider of sources.keys())
    if (collider.isValid()) simulation.removeCollider(collider, true);
  sources.clear();
}
