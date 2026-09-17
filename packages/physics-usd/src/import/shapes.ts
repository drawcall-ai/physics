import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
  SphereCollider,
  axisVector,
} from "@drawcall/physics";
import type { Collider, PhysicsMaterial } from "@drawcall/physics";
import { Mesh, Object3D, Quaternion, Vector3 } from "three";
import { numeric, token } from "./layer.js";
import type { Layer } from "./layer.js";

export function readShape(
  layer: Layer,
  path: string,
  type: string,
  object: Object3D,
  material: PhysicsMaterial,
): Collider {
  let collider: Collider;
  const scale = object.scale;
  if (Math.min(scale.x, scale.y, scale.z) <= 0)
    throw new Error(`Collider requires positive scale: ${path}`);
  const uniform = () => {
    if (
      Math.abs(scale.x - scale.y) > 1e-6 ||
      Math.abs(scale.x - scale.z) > 1e-6
    )
      throw new Error(`Nonuniform collider scale is unsupported: ${path}`);
    return scale.x;
  };
  if (type === "Cube") {
    const size = numeric(layer, path, "size", 2);
    collider = new BoxCollider({
      size: [size * scale.x, size * scale.y, size * scale.z],
    });
  } else if (type === "Sphere") {
    collider = new SphereCollider({
      radius: numeric(layer, path, "radius", 1) * uniform(),
    });
  } else if (type === "Capsule" || type === "Cylinder") {
    const factor = uniform();
    const radius = numeric(layer, path, "radius", 1) * factor;
    const height = numeric(layer, path, "height", 2) * factor;
    collider =
      type === "Capsule"
        ? new CapsuleCollider({ radius, height })
        : new CylinderCollider({ radius, height });
  } else if (type === "Mesh") {
    if (!(object instanceof Mesh))
      throw new Error(`Missing collider mesh geometry: ${path}`);
    const approximation = token(layer, path, "physics:approximation", "none");
    if (approximation !== "none" && approximation !== "convexHull")
      throw new Error(
        `Unsupported collision approximation ${approximation}: ${path}`,
      );
    const geometry = object.geometry.clone();
    geometry.scale(scale.x, scale.y, scale.z);
    collider = new MeshCollider({
      approximation: approximation === "none" ? "trimesh" : "convexHull",
    }).setGeometry(geometry);
  } else throw new Error(`Unsupported collision shape ${type}: ${path}`);
  collider.setMaterial(material);
  collider.position.copy(object.position);
  collider.quaternion.copy(object.quaternion);
  if (type === "Capsule" || type === "Cylinder") {
    const axis = token(layer, path, "axis", "Z");
    if (axis !== "X" && axis !== "Y" && axis !== "Z")
      throw new Error(`Invalid collider axis ${axis}`);
    collider.quaternion.multiply(
      new Quaternion().setFromUnitVectors(
        new Vector3(0, 1, 0),
        axisVector(axis),
      ),
    );
  }
  collider.name = object.name;
  return collider;
}
