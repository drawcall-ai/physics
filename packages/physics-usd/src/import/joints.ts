import { Matrix4, Quaternion, Vector3 } from "three";
import {
  DistanceJoint,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  SphericalJoint,
} from "@drawcall/physics";
import type {
  AxisJointOptions,
  Joint,
  JointOptions,
  RigidBody,
} from "@drawcall/physics";
import {
  attribute,
  boolean,
  numeric,
  numbers,
  target,
  token,
} from "./layer.js";
import type { Layer } from "./layer.js";

function frame(layer: Layer, path: string, index: number): Matrix4 {
  const p = numbers(layer, path, `physics:localPos${index}`) ?? [0, 0, 0];
  const q = numbers(layer, path, `physics:localRot${index}`) ?? [0, 0, 0, 1];
  const [x, y, z] = p;
  const [qx, qy, qz, qw] = q;
  if (
    p.length !== 3 ||
    q.length !== 4 ||
    x === undefined ||
    y === undefined ||
    z === undefined ||
    qx === undefined ||
    qy === undefined ||
    qz === undefined ||
    qw === undefined
  )
    throw new Error(`Invalid joint frame on ${path}`);
  return new Matrix4().compose(
    new Vector3(x, y, z),
    new Quaternion(qx, qy, qz, qw),
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
    const max = numeric(layer, path, "physics:maxDistance", -1);
    if (max < 0)
      throw new Error(
        `Distance joints require an explicit finite maximum distance: ${path}`,
      );
    const joint = new DistanceJoint(options);
    try {
      joint.setLimits([
        Math.max(0, numeric(layer, path, "physics:minDistance", -1)),
        max,
      ]);
      return joint;
    } catch (error) {
      joint.dispose();
      throw error;
    }
  }
  if (type !== "PhysicsRevoluteJoint" && type !== "PhysicsPrismaticJoint")
    throw new Error(`Unsupported USD joint ${type}`);
  const axis = token(layer, path, "physics:axis", "X");
  if (axis !== "X" && axis !== "Y" && axis !== "Z")
    throw new Error(`Invalid joint axis ${axis}`);
  const angular = type === "PhysicsRevoluteJoint";
  const factor = angular ? Math.PI / 180 : 1;
  const axisOptions: AxisJointOptions = { ...options, axis };
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
  const joint = angular
    ? new RevoluteJoint(axisOptions)
    : new PrismaticJoint(axisOptions);
  try {
    joint.setLimits(limits);
    return joint;
  } catch (error) {
    joint.dispose();
    throw error;
  }
}

export function readJoint(
  layer: Layer,
  path: string,
  type: string,
  bodies: Map<string, RigidBody>,
): Joint {
  const enabled = boolean(layer, path, "physics:jointEnabled", true);
  const collideConnected = boolean(
    layer,
    path,
    "physics:collisionEnabled",
    false,
  );
  const joint = createJoint(layer, path, type, bodies);
  joint.setEnabled(enabled);
  joint.setCollideConnected(collideConnected);
  return joint;
}
