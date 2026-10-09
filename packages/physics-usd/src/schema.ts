import {
  DistanceJoint,
  FixedJoint,
  GenericJoint,
  PrismaticJoint,
  RevoluteJoint,
  SphericalJoint,
  jointDofs,
} from "@drawcall/physics";
import type { Joint } from "@drawcall/physics";

interface JointSchema {
  type: string;
  joint: abstract new (...args: never[]) => Joint;
  /** PhysicsDriveAPI instance names the prim type accepts. */
  drives: readonly string[];
}

export const jointSchemas: readonly JointSchema[] = [
  { type: "PhysicsFixedJoint", joint: FixedJoint, drives: [] },
  { type: "PhysicsRevoluteJoint", joint: RevoluteJoint, drives: ["angular"] },
  { type: "PhysicsPrismaticJoint", joint: PrismaticJoint, drives: ["linear"] },
  // UsdPhysics defines no distance drive; the linear instance is an extension.
  { type: "PhysicsDistanceJoint", joint: DistanceJoint, drives: ["linear"] },
  { type: "PhysicsSphericalJoint", joint: SphericalJoint, drives: [] },
  { type: "PhysicsJoint", joint: GenericJoint, drives: jointDofs },
];

export const driveInstances = [
  ...new Set(jointSchemas.flatMap((schema) => schema.drives)),
];

export function schemaOf(joint: Joint): JointSchema {
  const schema = jointSchemas.find((entry) => joint instanceof entry.joint);
  if (!schema) throw new Error(`Unsupported joint ${joint.constructor.name}`);
  return schema;
}

export function schemaFor(type: string): JointSchema {
  const schema = jointSchemas.find((entry) => entry.type === type);
  if (!schema) throw new Error(`Unsupported USD joint ${type}`);
  return schema;
}

/** The single drive instance of a revolute, prismatic, or distance joint type. */
export function scalarDrive(schema: JointSchema): string {
  const [instance, ...rest] = schema.drives;
  if (!instance || rest.length)
    throw new Error(`${schema.type} has no single drive`);
  return instance;
}

/** USD authors angular drives and limits in degrees. */
export function isAngular(instance: string): boolean {
  return instance === "angular" || instance.startsWith("rot");
}
