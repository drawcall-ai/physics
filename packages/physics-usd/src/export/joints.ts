import { Matrix4, Quaternion, Vector3 } from "three";
import type { Object3D } from "three";
import {
  AxisJoint,
  DistanceJoint,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  SphericalJoint,
} from "@drawcall/physics";
import type { Joint } from "@drawcall/physics";
import { Prim } from "./prim.js";
import { tuple } from "./shapes.js";
const degrees = 180 / Math.PI;

export function writeJoint(
  joint: Joint,
  name: string,
  paths: Map<Object3D, string>,
): Prim {
  joint.validate();
  const type =
    joint instanceof FixedJoint
      ? "PhysicsFixedJoint"
      : joint instanceof RevoluteJoint
        ? "PhysicsRevoluteJoint"
        : joint instanceof PrismaticJoint
          ? "PhysicsPrismaticJoint"
          : joint instanceof SphericalJoint
            ? "PhysicsSphericalJoint"
            : joint instanceof DistanceJoint
              ? "PhysicsDistanceJoint"
              : undefined;
  if (!type) throw new Error(`Unsupported joint ${joint.constructor.name}`);
  const prim = new Prim(name, type);
  const body1 = paths.get(joint.options.body1);
  const body0 = joint.options.body0
    ? paths.get(joint.options.body0)
    : undefined;
  if (!body1 || (joint.options.body0 && !body0))
    throw new Error("Joint body missing from USD scene");
  if (body0) prim.properties.push(`rel physics:body0 = <${body0}>`);
  prim.properties.push(
    `rel physics:body1 = <${body1}>`,
    `bool physics:jointEnabled = ${joint.options.enabled ?? true}`,
    `bool physics:collisionEnabled = ${joint.options.collideConnected ?? false}`,
  );
  for (const index of [0, 1] as const) {
    const matrix = joint.getFrame(index, new Matrix4());
    const p = new Vector3().setFromMatrixPosition(matrix);
    const q = new Quaternion().setFromRotationMatrix(matrix);
    prim.properties.push(
      `point3f physics:localPos${index} = ${tuple([p.x, p.y, p.z])}`,
      `quatf physics:localRot${index} = ${tuple([q.w, q.x, q.y, q.z])}`,
    );
  }
  if (joint instanceof DistanceJoint)
    prim.properties.push(
      `float physics:minDistance = ${joint.options.limits[0]}`,
      `float physics:maxDistance = ${joint.options.limits[1]}`,
    );
  if (!(joint instanceof AxisJoint)) return prim;
  const factor = joint instanceof RevoluteJoint ? degrees : 1;
  prim.properties.push(
    `uniform token physics:axis = "${joint.options.axis ?? "Y"}"`,
  );
  if (joint.options.limits)
    prim.properties.push(
      `float physics:lowerLimit = ${joint.options.limits[0] * factor}`,
      `float physics:upperLimit = ${joint.options.limits[1] * factor}`,
    );
  if (!joint.options.drive) return prim;
  const drive = joint.options.drive;
  const axis = joint instanceof RevoluteJoint ? "angular" : "linear";
  const prefix = `drive:${axis}:physics:`;
  prim.schemas.push(`PhysicsDriveAPI:${axis}`);
  prim.properties.push(
    `uniform token ${prefix}type = "${drive.type ?? "force"}"`,
    `float ${prefix}targetPosition = ${(drive.targetPosition ?? 0) * factor}`,
    `float ${prefix}targetVelocity = ${(drive.targetVelocity ?? 0) * factor}`,
    `float ${prefix}stiffness = ${(drive.stiffness ?? 0) / factor}`,
    `float ${prefix}damping = ${(drive.damping ?? 0) / factor}`,
  );
  if (drive.maxForce !== undefined)
    prim.properties.push(`float ${prefix}maxForce = ${drive.maxForce}`);
  return prim;
}
