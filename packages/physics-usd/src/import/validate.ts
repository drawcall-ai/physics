import { jointDofs } from "@drawcall/physics";
import { PRIM_SPEC, attribute, numeric, schemas } from "./layer.js";
import type { Layer } from "./layer.js";

type Spec = Layer["specsByPath"][string];

const properties = new Set([
  "rigidBodyEnabled",
  "kinematicEnabled",
  "mass",
  "centerOfMass",
  "diagonalInertia",
  "principalAxes",
  "density",
  "velocity",
  "angularVelocity",
  "collisionEnabled",
  "approximation",
  "staticFriction",
  "dynamicFriction",
  "restitution",
  "jointEnabled",
  "body0",
  "body1",
  "localPos0",
  "localPos1",
  "localRot0",
  "localRot1",
  "axis",
  "lowerLimit",
  "upperLimit",
  "minDistance",
  "maxDistance",
  "gravityDirection",
  "gravityMagnitude",
]);
const supported = new Set([
  "PhysicsRigidBodyAPI",
  "PhysicsMassAPI",
  "PhysicsCollisionAPI",
  "PhysicsMeshCollisionAPI",
  "PhysicsMaterialAPI",
  "PhysicsDriveAPI:angular",
  "PhysicsDriveAPI:linear",
  ...jointDofs.map((dof) => `PhysicsDriveAPI:${dof}`),
  ...jointDofs.map((dof) => `PhysicsLimitAPI:${dof}`),
]);
const operations = new Set([
  "xformOp:transform",
  "xformOp:translate",
  "xformOp:translate:pivot",
  "xformOp:orient",
  "xformOp:scale",
  "xformOp:rotateXYZ",
  "xformOp:rotateX",
  "xformOp:rotateY",
  "xformOp:rotateZ",
]);
const driveProperty =
  /^drive:(angular|linear|transX|transY|transZ|rotX|rotY|rotZ):physics:(type|targetPosition|targetVelocity|stiffness|damping|maxForce)$/;
const limitProperty =
  /^limit:(transX|transY|transZ|rotX|rotY|rotZ):physics:(low|high)$/;

export function validate(layer: Layer): void {
  validateUnits(layer);
  let scenes = 0;
  for (const [path, spec] of Object.entries(layer.specsByPath)) {
    if (spec.specType === PRIM_SPEC) {
      validateTransform(path, spec);
      validateSchemas(layer, path, spec);
      if (spec.fields.typeName === "PhysicsScene" && ++scenes > 1)
        throw new Error("Multiple physics scenes are unsupported");
      validateColliderMass(layer, path);
    }
    validateProperty(layer, path);
    if (spec.fields.timeSamples !== undefined)
      throw new Error(`Animated physics properties are unsupported: ${path}`);
  }
}

function validateUnits(layer: Layer): void {
  const root = layer.specsByPath["/"]?.fields;
  if (
    root?.metersPerUnit !== 1 ||
    (root.kilogramsPerUnit ?? 1) !== 1 ||
    (root.upAxis ?? "Y") !== "Y"
  )
    throw new Error(
      "Physics USD import currently requires metersPerUnit=1, kilogramsPerUnit=1, and Y-up",
    );
}

function validateTransform(path: string, spec: Spec): void {
  const order = spec.fields.xformOpOrder;
  if (order === undefined) return;
  if (
    !Array.isArray(order) ||
    !order.every(
      (op) =>
        typeof op === "string" &&
        operations.has(op.startsWith("!invert!") ? op.slice(8) : op),
    )
  )
    throw new Error(`Unsupported USD transform operation on ${path}`);
}

function validateSchemas(layer: Layer, path: string, spec: Spec): void {
  const type = spec.fields.typeName;
  const driveAxes: readonly string[] =
    type === "PhysicsRevoluteJoint"
      ? ["angular"]
      : type === "PhysicsPrismaticJoint" || type === "PhysicsDistanceJoint"
        ? ["linear"]
        : type === "PhysicsJoint"
          ? jointDofs
          : [];
  for (const schema of schemas(layer, path)) {
    const [api, instance = ""] = schema.split(":");
    if (api === "PhysicsDriveAPI" && !driveAxes.includes(instance))
      throw new Error(`Unsupported USD drive schema ${schema} on prim ${path}`);
    if (api === "PhysicsLimitAPI" && type !== "PhysicsJoint")
      throw new Error(`Unsupported USD limit schema ${schema} on prim ${path}`);
    if (schema.startsWith("Physics") && !supported.has(schema))
      throw new Error(`Unsupported USD schema ${schema}: ${path}`);
  }
}

/** Mass on a collider prim is only allowed as a density hint; explicit mass belongs to the body. */
function validateColliderMass(layer: Layer, path: string): void {
  if (
    !schemas(layer, path).includes("PhysicsMassAPI") ||
    schemas(layer, path).includes("PhysicsRigidBodyAPI")
  )
    return;
  const explicit =
    numeric(layer, path, "physics:mass", 0) !== 0 ||
    ["centerOfMass", "diagonalInertia", "principalAxes"].some(
      (name) => attribute(layer, path, `physics:${name}`) !== undefined,
    );
  if (!explicit) return;
  for (
    let parent = path.slice(0, path.lastIndexOf("/"));
    parent;
    parent = parent.slice(0, parent.lastIndexOf("/"))
  )
    if (
      schemas(layer, parent).includes("PhysicsRigidBodyAPI") ||
      schemas(layer, parent).includes("PhysicsMassAPI")
    )
      throw new Error(
        `Per-collider explicit mass properties are unsupported: ${path}`,
      );
}

function validateProperty(layer: Layer, path: string): void {
  const property = path.split(".").slice(1).join(".");
  if (property.startsWith("physics:") && !properties.has(property.slice(8)))
    throw new Error(`Unsupported USD physics property ${path}`);
  const prim = path.split(".")[0] ?? "";
  const instance = property.split(":")[1];
  if (
    property.startsWith("drive:") &&
    (!driveProperty.test(property) ||
      !schemas(layer, prim).includes(`PhysicsDriveAPI:${instance}`))
  )
    throw new Error(
      `Unsupported USD drive property ${property} on prim ${prim}`,
    );
  if (
    property.startsWith("limit:") &&
    (!limitProperty.test(property) ||
      !schemas(layer, prim).includes(`PhysicsLimitAPI:${instance}`))
  )
    throw new Error(
      `Unsupported USD limit property ${property} on prim ${prim}`,
    );
  if (property.startsWith("physx"))
    throw new Error(`Unsupported USD physics property ${path}`);
}
