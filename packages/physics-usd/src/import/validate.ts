import { numeric, schemas } from "./layer.js";
import type { Layer } from "./layer.js";

export function validate(layer: Layer): void {
  const root = layer.specsByPath["/"]?.fields;
  if (
    root?.metersPerUnit !== 1 ||
    (root.kilogramsPerUnit ?? 1) !== 1 ||
    (root.upAxis ?? "Y") !== "Y"
  )
    throw new Error(
      "Physics USD import currently requires metersPerUnit=1, kilogramsPerUnit=1, and Y-up",
    );
  let scenes = 0;
  const properties = new Set([
    "rigidBodyEnabled",
    "kinematicEnabled",
    "mass",
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
  ]);
  for (const [path, spec] of Object.entries(layer.specsByPath)) {
    if (spec.specType === 6) {
      const order = spec.fields.xformOpOrder;
      if (order !== undefined) {
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
      const drive = schemas(layer, path).filter((schema) =>
        schema.startsWith("PhysicsDriveAPI:"),
      );
      const expected =
        spec.fields.typeName === "PhysicsRevoluteJoint"
          ? "PhysicsDriveAPI:angular"
          : spec.fields.typeName === "PhysicsPrismaticJoint"
            ? "PhysicsDriveAPI:linear"
            : undefined;
      if (drive.some((schema) => schema !== expected))
        throw new Error(`Unsupported drive axis on ${path}`);
      if (spec.fields.typeName === "PhysicsScene" && ++scenes > 1)
        throw new Error("Multiple physics scenes are unsupported");
      for (const schema of schemas(layer, path))
        if (schema.startsWith("Physics") && !supported.has(schema))
          throw new Error(`Unsupported USD schema ${schema}: ${path}`);
    }
    const property = path.split(".").slice(1).join(".");
    if (property.startsWith("physics:") && !properties.has(property.slice(8)))
      throw new Error(`Unsupported USD physics property ${path}`);
    if (
      property.startsWith("drive:") &&
      !/^drive:(angular|linear):physics:(type|targetPosition|targetVelocity|stiffness|damping|maxForce)$/.test(
        property,
      )
    )
      throw new Error(`Unsupported USD drive property ${path}`);
    if (
      schemas(layer, path).includes("PhysicsMassAPI") &&
      !schemas(layer, path).includes("PhysicsRigidBodyAPI") &&
      numeric(layer, path, "physics:mass", 0) !== 0
    )
      throw new Error(`Per-collider explicit mass is unsupported: ${path}`);
    if (
      /\.(physics:(breakForce|breakTorque|centerOfMass|diagonalInertia|principalAxes|simulationOwner|filteredPairs)|physx)/.test(
        path,
      )
    )
      throw new Error(`Unsupported USD physics property ${path}`);
    if (spec.fields.timeSamples !== undefined)
      throw new Error(`Animated physics properties are unsupported: ${path}`);
  }
}
