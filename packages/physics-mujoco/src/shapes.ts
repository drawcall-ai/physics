import {
  RigidBody,
  resolveCollider,
  resolveCollisionGroups,
  type Trigger,
  type CollisionGroups,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import { terrain } from "./terrain.js";
import { placement, name } from "./values.js";

export interface Geometry {
  name: string;
  owner: RigidBody | Trigger;
  source: Object3D;
  groups: CollisionGroups;
  friction: number;
  restitution: number;
}
export interface Shapes {
  xml: string;
  assets: string[];
  geometries: Geometry[];
}
export function shapes(owner: RigidBody | Trigger): Shapes {
  const assets: string[] = [],
    geometries: Geometry[] = [],
    xml: string[] = [];
  const colliders = owner.getColliders();
  const unitDensity =
    owner instanceof RigidBody &&
    owner.options.mass !== undefined &&
    !owner.options.centerOfMass &&
    colliders.every((collider) => owner.getMaterial(collider).density === 0);
  for (const collider of colliders) {
    const part = resolveCollider(owner, collider);
    const { shape } = part;
    const material =
      owner instanceof RigidBody ? owner.getMaterial(collider) : undefined;
    if (material && material.staticFriction !== material.dynamicFriction)
      throw new Error("MuJoCo requires equal static and dynamic friction");
    const prefix = `${name(owner)}g${geometries.length}`;
    const add = (suffix: string, attributes: string, matrix = part.matrix) => {
      const key = prefix + suffix;
      geometries.push({
        name: key,
        owner,
        source: collider.source,
        groups: resolveCollisionGroups(collider, owner),
        friction: material?.dynamicFriction ?? 0,
        restitution: material?.restitution ?? 0,
      });
      xml.push(
        `<geom name="${key}" ${placement(matrix)} ${attributes} density="${unitDensity ? 1 : (material?.density ?? 0)}" contype="0" conaffinity="0"/>`,
      );
    };
    if (shape.kind === "box")
      add("", `type="box" size="${shape.size.map((v) => v / 2).join(" ")}"`);
    else if (shape.kind === "sphere")
      add("", `type="sphere" size="${shape.radius}"`);
    else if (shape.kind === "capsule" || shape.kind === "cylinder") {
      const matrix = part.matrix
        .clone()
        .multiply(
          new Matrix4().makeRotationFromQuaternion(
            new Quaternion().setFromAxisAngle(
              new Vector3(1, 0, 0),
              Math.PI / 2,
            ),
          ),
        );
      add(
        "",
        `type="${shape.kind}" size="${shape.radius} ${shape.height / 2}"`,
        matrix,
      );
    } else {
      const positions = shape.geometry.getAttribute("position");
      const heightfield =
        shape.approximation === "trimesh"
          ? terrain(shape.geometry, prefix)
          : undefined;
      if (heightfield) {
        assets.push(heightfield.asset);
        add(
          "",
          `type="hfield" hfield="${prefix}"`,
          part.matrix.clone().multiply(heightfield.matrix),
        );
      } else if (shape.approximation === "convexHull") {
        const vertices: number[] = [];
        for (let i = 0; i < positions.count; i++)
          vertices.push(
            positions.getX(i),
            positions.getY(i),
            positions.getZ(i),
          );
        assets.push(`<mesh name="${prefix}" vertex="${vertices.join(" ")}"/>`);
        add("", `type="mesh" mesh="${prefix}"`);
      } else {
        // MuJoCo collides mesh convex hulls. Individual thin triangular prisms preserve concavities.
        const indices = shape.geometry.index;
        for (let i = 0; i < (indices?.count ?? positions.count); i += 3) {
          const points = [0, 1, 2].map((j) =>
            new Vector3().fromBufferAttribute(
              positions,
              indices ? indices.getX(i + j) : i + j,
            ),
          );
          const [a, b, c] = points;
          if (!a || !b || !c) throw new Error("Incomplete mesh triangle");
          const normal = b.clone().sub(a).cross(c.clone().sub(a));
          if (normal.lengthSq() === 0) continue;
          normal.normalize().multiplyScalar(0.001);
          const vertices = [
            ...points,
            ...points.map((p) => p.clone().sub(normal)),
          ].flatMap((p) => p.toArray());
          const key = `${prefix}t${i}`;
          assets.push(`<mesh name="${key}" vertex="${vertices.join(" ")}"/>`);
          add(`t${i}`, `type="mesh" mesh="${key}"`);
        }
      }
      shape.geometry.dispose();
    }
  }
  return { xml: xml.join(""), assets, geometries };
}
export function matches(a: CollisionGroups, b: CollisionGroups): boolean {
  return (a.membership & b.filter) !== 0 && (b.membership & a.filter) !== 0;
}
