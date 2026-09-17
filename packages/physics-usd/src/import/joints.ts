import { Matrix4, Quaternion, Vector3 } from "three";
import {
  DistanceJoint,
  GenericJoint,
  JointDrive,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  ScalarJoint,
  SphericalJoint,
  jointDofs,
} from "@drawcall/physics";
import type {
  AxisJointOptions,
  DofMotion,
  Joint,
  JointDof,
  JointOptions,
  RigidBody,
} from "@drawcall/physics";
import {
  attribute,
  boolean,
  numeric,
  numbers,
  target,
  schemas,
  token,
} from "./layer.js";
import type { Layer } from "./layer.js";

function frame(layer: Layer, path: string, index: number): Matrix4 {
  const p = numbers(layer, path, `physics:localPos${index}`) ?? [0, 0, 0];
  const q = numbers(layer, path, `physics:localRot${index}`) ?? [0, 0, 0, 1];
  if (p.length !== 3 || q.length !== 4)
    throw new Error(`Invalid joint frame on ${path}`);
  return new Matrix4().compose(
    new Vector3().fromArray(p),
    new Quaternion().fromArray(q),
    new Vector3(1, 1, 1),
  );
}

function createJoint(
  layer: Layer,
  path: string,
  type: string,
  bodies: Map<string, RigidBody>,
): Joint {
  const reference = (name: string) => {
    const relation = target(layer, path, name);
    if (!relation) return null;
    const body = bodies.get(relation);
    if (!body)
      throw new Error(`Joint ${path} references missing body ${relation}`);
    return body;
  };
  const body0 = reference("physics:body0");
  const body1 = reference("physics:body1");
  if (!body1)
    throw new Error(
      `Joint ${path} requires body1; world anchoring is supported through body0`,
    );
  const options: JointOptions = {
    body0,
    body1,
    frame0: frame(layer, path, 0),
    frame1: frame(layer, path, 1),
  };
  if (type === "PhysicsFixedJoint") return new FixedJoint(options);
  if (type === "PhysicsSphericalJoint") {
    for (const name of ["physics:coneAngle0Limit", "physics:coneAngle1Limit"]) {
      if (attribute(layer, path, name) !== undefined)
        throw new Error(`Spherical cone limits are not supported: ${path}`);
    }
    return new SphericalJoint(options);
  }
  if (type === "PhysicsDistanceJoint") {
    // USD's negative defaults mean unlimited.
    const max = numeric(layer, path, "physics:maxDistance", -1);
    return new DistanceJoint({
      ...options,
      limits: [
        Math.max(0, numeric(layer, path, "physics:minDistance", -1)),
        max < 0 ? Infinity : max,
      ],
    });
  }

  if (type === "PhysicsJoint") {
    const dofs: Partial<Record<JointDof, DofMotion>> = {};
    for (const axis of jointDofs) {
      if (!schemas(layer, path).includes(`PhysicsLimitAPI:${axis}`)) {
        dofs[axis] = "free";
        continue;
      }
      const scale = axis.startsWith("rot") ? Math.PI / 180 : 1;
      const low = numeric(layer, path, `limit:${axis}:physics:low`, -Infinity);
      const high = numeric(layer, path, `limit:${axis}:physics:high`, Infinity);
      dofs[axis] = low > high ? "locked" : [low * scale, high * scale];
    }
    return new GenericJoint({ ...options, dofs });
  }
  if (type !== "PhysicsRevoluteJoint" && type !== "PhysicsPrismaticJoint")
    throw new Error(`Unsupported USD joint ${type}`);
  const axis = token(layer, path, "physics:axis", "X");
  if (axis !== "X" && axis !== "Y" && axis !== "Z")
    throw new Error(`Invalid joint axis ${axis}`);
  const angular = type === "PhysicsRevoluteJoint";
  const factor = angular ? Math.PI / 180 : 1;
  let limits: readonly [number, number] | undefined;
  const lower = attribute(layer, path, "physics:lowerLimit");
  const upper = attribute(layer, path, "physics:upperLimit");
  if (lower !== undefined || upper !== undefined) {
    if (lower === undefined || upper === undefined)
      throw new Error(`Both finite joint limits are required: ${path}`);
    limits = [
      numeric(layer, path, "physics:lowerLimit", 0) * factor,
      numeric(layer, path, "physics:upperLimit", 0) * factor,
    ];
  }
  const axisOptions: AxisJointOptions = { ...options, axis, limits };
  return angular
    ? new RevoluteJoint(axisOptions)
    : new PrismaticJoint(axisOptions);
}

function readDrive(
  layer: Layer,
  path: string,
  instance: string,
  angular: boolean,
): JointDrive | undefined {
  if (!schemas(layer, path).includes(`PhysicsDriveAPI:${instance}`))
    return undefined;
  const prefix = `drive:${instance}:physics:`;
  const model = token(layer, path, `${prefix}type`, "force");
  if (model !== "force" && model !== "acceleration")
    throw new Error(`Unsupported USD drive type ${model}: ${path}`);
  const factor = angular ? Math.PI / 180 : 1;
  const maxForce = attribute(layer, path, `${prefix}maxForce`);
  return new JointDrive({
    model,
    stiffness: numeric(layer, path, `${prefix}stiffness`, 0) / factor,
    damping: numeric(layer, path, `${prefix}damping`, 0) / factor,
    maxForce:
      maxForce === undefined || maxForce === Infinity
        ? undefined
        : numeric(layer, path, `${prefix}maxForce`, 0),
  }).setTarget({
    position: numeric(layer, path, `${prefix}targetPosition`, 0) * factor,
    velocity: numeric(layer, path, `${prefix}targetVelocity`, 0) * factor,
  });
}

export function readJoint(
  layer: Layer,
  path: string,
  type: string,
  bodies: Map<string, RigidBody>,
): Joint {
  const joint = createJoint(layer, path, type, bodies);
  try {
    joint.setEnabled(boolean(layer, path, "physics:jointEnabled", true));
    joint.setCollideConnected(
      boolean(layer, path, "physics:collisionEnabled", false),
    );
    if (joint instanceof ScalarJoint) {
      const angular = joint instanceof RevoluteJoint;
      joint.setDrive(
        readDrive(layer, path, angular ? "angular" : "linear", angular),
      );
    }
    if (joint instanceof GenericJoint)
      for (const axis of jointDofs)
        joint.setDrive(
          axis,
          readDrive(layer, path, axis, axis.startsWith("rot")),
        );
    return joint;
  } catch (error) {
    joint.dispose();
    throw error;
  }
}
