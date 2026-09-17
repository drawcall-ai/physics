import { Mesh, Object3D, Quaternion, Vector3 } from "three";
import {
  type PhysicsMaterial,
  RigidBody,
  splitTransform,
} from "@drawcall/physics";
import type { PhysicsWorld, MassProperties, Vec3 } from "@drawcall/physics";
import { attribute, numeric, numbers, schemas, target } from "./layer.js";
import type { Layer } from "./layer.js";

export function vector(
  layer: Layer,
  path: string,
  name: string,
  fallback: Vec3,
): Vec3 {
  const values = numbers(layer, path, name);
  if (!values) return fallback;
  if (values.length !== 3) throw new Error(`Expected vector ${path}.${name}`);
  return new Vector3().fromArray(values).toArray();
}

export function wrapBody(
  object: Object3D,
  world: PhysicsWorld,
  type: "static" | "dynamic" | "kinematic",
  mass: MassProperties = {},
): RigidBody {
  const parent = object.parent;
  if (!parent) throw new Error("Cannot reconstruct an orphan rigid body");
  const body = new RigidBody({ ...mass, world, type, colliders: false });
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
  // UsdPhysics: an unbound collider uses the default material, which is frictionless.
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
  // UsdPhysics: a zero density means the scene default of 1000 kg/m³.
  if (material.density === 0) material.density = 1000;
  return material;
}

export function massProperties(
  layer: Layer,
  path: string,
  object: Object3D,
): MassProperties {
  const mass = numeric(layer, path, "physics:mass", 0);
  const optionalVector = (name: string) =>
    attribute(layer, path, name) === undefined
      ? undefined
      : vector(layer, path, name, [0, 0, 0]);
  let principalAxes: MassProperties["principalAxes"];
  const axes = numbers(layer, path, "physics:principalAxes");
  if (axes) {
    if (axes.length !== 4)
      throw new Error(`Expected quaternion ${path}.physics:principalAxes`);
    if (axes.some((value) => value !== 0))
      principalAxes = new Quaternion().fromArray(axes).toArray();
  }
  object.updateWorldMatrix(true, false);
  const scale = splitTransform(object.matrixWorld).scale;
  const center = optionalVector("physics:centerOfMass");
  const inertia = optionalVector("physics:diagonalInertia");
  const diagonalInertia = inertia?.some((value) => value !== 0)
    ? inertia
    : undefined;
  if (!center && !diagonalInertia && !principalAxes)
    return { mass: mass === 0 ? undefined : mass };
  if (mass === 0 || !center || !diagonalInertia)
    throw new Error(
      `Explicit mass properties require mass, centerOfMass and diagonalInertia: ${path}`,
    );
  return {
    mass,
    centerOfMass: [
      center[0] * scale.x,
      center[1] * scale.y,
      center[2] * scale.z,
    ],
    diagonalInertia,
    principalAxes,
  };
}
