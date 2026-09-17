import { resolveCollider } from "@drawcall/physics";
import type { Collider, PhysicsMaterial, RigidBody } from "@drawcall/physics";
import { degrees } from "../units.js";
import type { Hierarchy } from "./hierarchy.js";
import { Prim } from "./prim.js";
import { shapePrim, tuple } from "./shapes.js";

/** One material prim per distinct authored material, shared by every collider that resolves to it. */
export class Materials {
  readonly scope = new Prim("PhysicsMaterials", "Scope");
  private readonly paths = new Map<PhysicsMaterial | undefined, string>();

  path(body: RigidBody, collider: Collider): string {
    const key = collider.material ?? body.material;
    const existing = this.paths.get(key);
    if (existing) return existing;
    const material = body.getMaterial(collider);
    const prim = new Prim(`Material${this.paths.size}`, "Material");
    prim.schemas.push("PhysicsMaterialAPI");
    for (const property of [
      "staticFriction",
      "dynamicFriction",
      "restitution",
      "density",
    ] as const)
      prim.properties.push(`float physics:${property} = ${material[property]}`);
    this.scope.children.push(prim);
    const path = `/Root/PhysicsMaterials/${prim.name}`;
    this.paths.set(key, path);
    return path;
  }
}

export function writeBody(
  hierarchy: Hierarchy,
  body: RigidBody,
  materials: Materials,
): void {
  const colliders = body.getColliders();
  const prim = hierarchy.prim(body);
  if (
    body.linearDamping !== 0 ||
    body.angularDamping !== 0 ||
    body.gravityScale !== 1 ||
    !(body.options.canSleep ?? true)
  )
    throw new Error(
      "Core USD Physics cannot represent damping, gravity scale, or sleep policy overrides",
    );
  prim.schemas.push("PhysicsMassAPI");
  if (body.bodyType !== "static") {
    const velocity = body.getVelocity();
    prim.schemas.push("PhysicsRigidBodyAPI");
    prim.properties.push(
      "bool physics:rigidBodyEnabled = true",
      `bool physics:kinematicEnabled = ${body.bodyType === "kinematic"}`,
      `vector3f physics:velocity = ${tuple(velocity.linear.toArray())}`,
      `vector3f physics:angularVelocity = ${tuple(
        velocity.angular.toArray().map((value) => value * degrees),
      )}`,
    );
  }
  if (body.options.mass !== undefined)
    prim.properties.push(`float physics:mass = ${body.options.mass}`);
  for (const name of ["centerOfMass", "diagonalInertia"] as const) {
    const value = body.options[name];
    if (value)
      prim.properties.push(
        `${name === "centerOfMass" ? "point3f" : "float3"} physics:${name} = ${tuple(value)}`,
      );
  }
  const axes = body.options.principalAxes;
  if (axes)
    prim.properties.push(
      `quatf physics:principalAxes = ${tuple([axes[3], axes[0], axes[1], axes[2]])}`,
    );
  for (const [index, collider] of colliders.entries())
    prim.children.push(colliderPrim(body, collider, index, materials));
}

function colliderPrim(
  body: RigidBody,
  collider: Collider,
  index: number,
  materials: Materials,
): Prim {
  if (collider.collisionGroups)
    throw new Error(
      "USD collision filter conversion is not implemented; remove collisionGroups or export without physics",
    );
  if (collider.sensor)
    throw new Error("Core USD Physics cannot represent sensors");
  const materialPath = materials.path(body, collider);
  const resolved = resolveCollider(body, collider);
  const prim = shapePrim(`Collider${index}`, resolved.shape, resolved.matrix);
  prim.schemas.push("PhysicsCollisionAPI", "MaterialBindingAPI");
  prim.properties.push(
    "bool physics:collisionEnabled = true",
    'token visibility = "invisible"',
    `rel material:binding:physics = <${materialPath}>`,
  );
  return prim;
}
