import { Matrix4, Quaternion, Vector3 } from "three";
import type { Object3D } from "three";
import {
  AxisJoint,
  DistanceJoint,
  FixedJoint,
  GenericJoint,
  PrismaticJoint,
  RevoluteJoint,
  ScalarJoint,
  SphericalJoint,
  jointDofs,
} from "@drawcall/physics";
import type { Joint, JointDrive } from "@drawcall/physics";
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
          : joint instanceof DistanceJoint
            ? "PhysicsDistanceJoint"
            : joint instanceof SphericalJoint
              ? "PhysicsSphericalJoint"
              : joint instanceof GenericJoint
                ? "PhysicsJoint"
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
    `bool physics:jointEnabled = ${joint.enabled}`,
    `bool physics:collisionEnabled = ${joint.collideConnected}`,
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
  // USD's -1 maximum means unlimited.
  if (joint instanceof DistanceJoint)
    prim.properties.push(
      `float physics:minDistance = ${joint.limits[0]}`,
      `float physics:maxDistance = ${joint.limits[1] === Infinity ? -1 : joint.limits[1]}`,
    );
  const factor = joint instanceof RevoluteJoint ? degrees : 1;
  if (joint instanceof AxisJoint) {
    prim.properties.push(
      `uniform token physics:axis = "${joint.options.axis}"`,
    );
    if (joint.limits)
      prim.properties.push(
        `float physics:lowerLimit = ${joint.limits[0] * factor}`,
        `float physics:upperLimit = ${joint.limits[1] * factor}`,
      );
  }
  if (joint instanceof GenericJoint) {
    for (const axis of jointDofs) {
      const motion = joint.dofs[axis];
      const scale = axis.startsWith("rot") ? degrees : 1;
      // UsdPhysics: no limit means free, and a lower limit above the upper one locks the axis.
      if (motion !== "free") {
        prim.schemas.push(`PhysicsLimitAPI:${axis}`);
        const [low, high] =
          motion === "locked"
            ? [1, -1]
            : [motion[0] * scale, motion[1] * scale];
        prim.properties.push(
          `float limit:${axis}:physics:low = ${low}`,
          `float limit:${axis}:physics:high = ${high}`,
        );
      }
      const drive = joint.getDrive(axis);
      if (drive) writeDrive(prim, axis, drive, scale);
    }
    return prim;
  }
  if (joint instanceof ScalarJoint && joint.drive)
    writeDrive(
      prim,
      joint instanceof RevoluteJoint ? "angular" : "linear",
      joint.drive,
      factor,
    );
  return prim;
}

function writeDrive(
  prim: Prim,
  instance: string,
  drive: JointDrive,
  factor: number,
): void {
  if (!drive.target)
    throw new Error(
      "USD PhysicsDriveAPI cannot represent an untargeted drive; detach it before export",
    );
  if (drive.target.effort !== 0)
    throw new Error(
      "USD PhysicsDriveAPI has no effort term; export drives without effort",
    );
  const prefix = `drive:${instance}:physics:`;
  prim.schemas.push(`PhysicsDriveAPI:${instance}`);
  prim.properties.push(
    `uniform token ${prefix}type = "${drive.options.model ?? "force"}"`,
    `float ${prefix}stiffness = ${(drive.options.stiffness ?? 0) / factor}`,
    `float ${prefix}damping = ${(drive.options.damping ?? 0) / factor}`,
    `float ${prefix}targetPosition = ${drive.target.position * factor}`,
    `float ${prefix}targetVelocity = ${drive.target.velocity * factor}`,
  );
  if (drive.options.maxForce !== undefined)
    prim.properties.push(`float ${prefix}maxForce = ${drive.options.maxForce}`);
}
