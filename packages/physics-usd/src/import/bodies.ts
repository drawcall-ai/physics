import { Mesh, Object3D } from "three";
import {
  type PhysicsMaterial,
  RigidBody,
  splitTransform,
} from "@drawcall/physics";
import type { PhysicsWorld, RigidBodyOptions, Vec3 } from "@drawcall/physics";
import { numeric, numbers, schemas, target } from "./layer.js";
import type { Layer } from "./layer.js";

export function vector(
  layer: Layer,
  path: string,
  name: string,
  fallback: Vec3,
): Vec3 {
  const values = numbers(layer, path, name);
  if (!values) return fallback;
  const [x, y, z] = values;
  if (
    values.length !== 3 ||
    x === undefined ||
    y === undefined ||
    z === undefined
  )
    throw new Error(`Expected vector ${path}.${name}`);
  return [x, y, z];
}

export function wrapBody(
  object: Object3D,
  world: PhysicsWorld,
  type: "static" | "dynamic" | "kinematic",
  mass: Partial<RigidBodyOptions> = {},
): RigidBody {
  const parent = object.parent;
  if (!parent) throw new Error("Cannot reconstruct an orphan rigid body");
  const body = new RigidBody({ ...mass, world, colliders: false }).setType(
    type,
  );
  body.name = object.name;
  body.position.copy(object.position);
  body.quaternion.copy(object.quaternion);
  body.scale.copy(object.scale);
  body.visible = object.visible;
  parent.add(body);
  if (object instanceof Mesh) {
    body.add(object);
    object.position.set(0, 0, 0);
    object.quaternion.identity();
    object.scale.set(1, 1, 1);
  } else {
    body.add(...object.children);
    object.removeFromParent();
  }
  return body;
}

export function ancestorBody(object: Object3D): RigidBody | undefined {
  let parent = object.parent;
  while (parent) {
    if (parent instanceof RigidBody) return parent;
    parent = parent.parent;
  }
  return undefined;
}

export function materialFor(
  layer: Layer,
  path: string,
  cache: Map<string, PhysicsMaterial>,
): PhysicsMaterial {
  let current = path;
  let material: PhysicsMaterial | undefined;
  let density: number | undefined;
  while (current) {
    if (
      density === undefined &&
      schemas(layer, current).includes("PhysicsMassAPI")
    ) {
      const value = numeric(layer, current, "physics:density", 0);
      if (value !== 0) density = value;
    }
    const binding = target(layer, current, "material:binding:physics");
    if (!material && binding) {
      material = cache.get(binding);
      if (!material) {
        material = readMaterial(layer, binding);
        cache.set(binding, material);
      }
    }
    current = current.slice(0, current.lastIndexOf("/"));
  }
  material ??= { staticFriction: 0, dynamicFriction: 0 };
  if (density !== undefined && density !== material.density)
    material = { ...material, density };
  return material;
}

function readMaterial(layer: Layer, path: string): PhysicsMaterial {
  if (!schemas(layer, path).includes("PhysicsMaterialAPI"))
    throw new Error(`Physics material schema missing: ${path}`);
  const material = {
    staticFriction: numeric(layer, path, "physics:staticFriction", 0),
    dynamicFriction: numeric(layer, path, "physics:dynamicFriction", 0),
    restitution: numeric(layer, path, "physics:restitution", 0),
    density: numeric(layer, path, "physics:density", 1000),
  };
  if (material.density === 0) material.density = 1000;
  return material;
}

export function massProperties(
  layer: Layer,
  path: string,
  object: Object3D,
): Partial<RigidBodyOptions> {
  const mass = numeric(layer, path, "physics:mass", 0);
  const optionalVector = (name: string) =>
    numbers(layer, path, name) === undefined
      ? undefined
      : vector(layer, path, name, [0, 0, 0]);
  let principalAxes: RigidBodyOptions["principalAxes"];
  const axes = numbers(layer, path, "physics:principalAxes");
  if (axes) {
    const [x, y, z, w] = axes;
    if (
      axes.length !== 4 ||
      x === undefined ||
      y === undefined ||
      z === undefined ||
      w === undefined
    )
      throw new Error(`Expected quaternion ${path}.physics:principalAxes`);
    if (axes.some((value) => value !== 0)) principalAxes = [x, y, z, w];
  }
  object.updateWorldMatrix(true, false);
  const scale = splitTransform(object.matrixWorld).scale;
  const center = optionalVector("physics:centerOfMass");
  const inertia = optionalVector("physics:diagonalInertia");
  return {
    mass: mass === 0 ? undefined : mass,
    centerOfMass: center && [
      center[0] * scale.x,
      center[1] * scale.y,
      center[2] * scale.z,
    ],
    diagonalInertia: inertia?.some((value) => value !== 0)
      ? inertia
      : undefined,
    principalAxes,
  };
}
