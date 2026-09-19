import {
  RigidBody,
  resolveCollider,
  resolveCollisionGroups,
  type Trigger,
  type CollisionGroups,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import type { Meshes } from "./meshes.js";
import { heightfield } from "./heightfield.js";
import { placement, name } from "../values.js";

export interface Geometry {
  name: string;
  owner: RigidBody | Trigger;
  source: Object3D;
  groups: CollisionGroups;
  friction: number;
  restitution: number;
}
interface Shapes {
  xml: string;
  assets: string[];
  geometries: Geometry[];
}
export function shapes(owner: RigidBody | Trigger, meshes?: Meshes): Shapes {
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
    try {
      const material =
        owner instanceof RigidBody ? owner.getMaterial(collider) : undefined;
      if (material && material.staticFriction !== material.dynamicFriction)
        throw new Error("MuJoCo requires equal static and dynamic friction");
      const prefix = `${name(owner)}g${geometries.length}`;
      const add = (
        suffix: string,
        attributes: string,
        matrix = part.matrix,
      ) => {
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
        const prepared = meshes?.get(owner, collider)?.shape;
        if (prepared?.kind === "heightfield") {
          const field = heightfield(shape.geometry, prefix);
          if (!field)
            throw new Error(
              "Prepared heightfield is not supported at this scale",
            );
          assets.push(field.asset);
          add(
            "",
            `type="hfield" hfield="${prefix}"`,
            part.matrix.clone().multiply(field.matrix),
          );
          continue;
        }
        const hulls: number[][] = [];
        if (prepared?.kind === "compound") {
          for (const hull of prepared.hulls)
            hulls.push(
              hull.map((value, i) => value * part.scale.getComponent(i % 3)),
            );
        } else {
          const positions = shape.geometry.getAttribute("position");
          const vertices: number[] = [];
          for (let i = 0; i < positions.count; i++)
            vertices.push(
              positions.getX(i),
              positions.getY(i),
              positions.getZ(i),
            );
          hulls.push(vertices);
        }
        for (const [index, hull] of hulls.entries()) {
          const key = `${prefix}h${index}`;
          assets.push(`<mesh name="${key}" vertex="${hull.join(" ")}"/>`);
          add(`h${index}`, `type="mesh" mesh="${key}"`);
        }
      }
    } finally {
      if (shape.kind === "mesh") shape.geometry.dispose();
    }
  }
  return { xml: xml.join(""), assets, geometries };
}
export function matches(a: CollisionGroups, b: CollisionGroups): boolean {
  return (a.membership & b.filter) !== 0 && (b.membership & a.filter) !== 0;
}
