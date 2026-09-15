import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import { AxisJoint, PrismaticJoint, FixedJoint, type Joint } from "./joints.js";
import { splitTransform } from "./transforms.js";
import type { PhysicsVelocity, JointMeasurements } from "./world.js";

/** Authored velocity is independent of immutable creation options and backend reset snapshots. */
const velocities = new WeakMap<RigidBody, PhysicsVelocity>();
export function authoredVelocity(object: RigidBody): PhysicsVelocity {
  const value = velocities.get(object);
  return {
    linear: value?.linear.clone() ?? new Vector3(),
    angular: value?.angular.clone() ?? new Vector3(),
  };
}
export function setAuthoredVelocity(
  object: RigidBody,
  value: Partial<PhysicsVelocity>,
): void {
  const next = authoredVelocity(object);
  if (value.linear) next.linear.copy(value.linear);
  if (value.angular) next.angular.copy(value.angular);
  velocities.set(object, next);
}
export function validateVector(value: Vector3): void {
  if (![value.x, value.y, value.z].every(Number.isFinite))
    throw new Error("Physics vectors must be finite");
}
export function setWorldPose(object: RigidBody, pose: Matrix4): void {
  object.updateWorldMatrix(true, false);
  const scale = splitTransform(object.matrixWorld).scale;
  const matrix = pose.clone().scale(scale);
  if (object.parent)
    matrix.premultiply(object.parent.matrixWorld.clone().invert());
  splitTransform(matrix);
  matrix.decompose(object.position, object.quaternion, object.scale);
  object.updateMatrix();
  object.updateMatrixWorld(true);
}
export function authoredJointState(
  object: Joint,
  frames?: readonly [Matrix4, Matrix4],
  velocityAtPoint: (
    body: RigidBody,
    point: Vector3,
  ) => Vector3 = authoredVelocityAtPoint,
): JointMeasurements {
  object.validate();
  const options = object.options;
  const frame = (index: 0 | 1): Matrix4 => {
    const body = index === 0 ? options.body0 : options.body1;
    const matrix = frames
      ? frames[index].clone()
      : object.getFrame(index, new Matrix4());
    // Spherical and distance constraints use anchors, without frame orientation.
    if (!(object instanceof AxisJoint || object instanceof FixedJoint))
      matrix.makeTranslation(new Vector3().setFromMatrixPosition(matrix));
    if (body) matrix.premultiply(splitTransform(body.matrixWorld).pose);
    return matrix;
  };
  const a = frame(0),
    b = frame(1);
  if (object instanceof AxisJoint) {
    const axis =
      object.options.axis === "X"
        ? new Vector3(1, 0, 0)
        : object.options.axis === "Z"
          ? new Vector3(0, 0, 1)
          : new Vector3(0, 1, 0);
    const rotation = new Matrix4().makeRotationFromQuaternion(
      new Quaternion().setFromUnitVectors(new Vector3(1, 0, 0), axis),
    );
    a.multiply(rotation);
    b.multiply(rotation);
  }
  const body0 = options.body0;
  const body1 = options.body1;
  const velocity0 = body0?.getVelocity() ?? {
    linear: new Vector3(),
    angular: new Vector3(),
  };
  const velocity1 = body1.getVelocity();
  const anchor0 = new Vector3().setFromMatrixPosition(a);
  const anchor1 = new Vector3().setFromMatrixPosition(b);
  const linear0 =
    object instanceof PrismaticJoint && body0
      ? velocityAtPoint(body0, anchor0)
      : new Vector3();
  const linear1 =
    object instanceof PrismaticJoint
      ? velocityAtPoint(body1, anchor1)
      : new Vector3();
  return jointState(
    a,
    b,
    velocity0.angular,
    velocity1.angular,
    linear0,
    linear1,
  );
}

/** Construction-time point velocity can use an explicit COM, but never infer one. */
export function authoredVelocityAtPoint(
  body: RigidBody,
  point: Vector3,
): Vector3 {
  const { linear, angular } = body.getVelocity();
  if (angular.lengthSq() === 0) return linear;
  const center = body.options.centerOfMass;
  if (!center)
    throw new Error(
      "Prismatic velocity needs prepared mass properties: finish assembly and call world.update(0), or supply complete explicit mass properties for authoring",
    );
  const worldCenter = new Vector3(...center).applyMatrix4(
    splitTransform(body.matrixWorld).pose,
  );
  return linear.add(angular.cross(point.clone().sub(worldCenter)));
}

/** Raw frame measurements used by simulation adapters. Angles are wrapped to [-pi, pi]. */
export function jointState(
  a: Matrix4,
  b: Matrix4,
  angular0: Vector3,
  angular1: Vector3,
  linear0 = new Vector3(),
  linear1 = new Vector3(),
) {
  const rotation = new Quaternion().setFromRotationMatrix(a);
  const relative = rotation
    .clone()
    .invert()
    .multiply(new Quaternion().setFromRotationMatrix(b));
  const axis = new Vector3(1, 0, 0).applyQuaternion(rotation);
  const translation = new Vector3()
    .setFromMatrixPosition(b)
    .sub(new Vector3().setFromMatrixPosition(a));
  const angle = 2 * Math.atan2(relative.x, relative.w);
  return {
    angle: Math.atan2(Math.sin(angle), Math.cos(angle)),
    angularVelocity: angular1.clone().sub(angular0).dot(axis),
    position: translation.dot(axis),
    velocity:
      linear1.clone().sub(linear0).dot(axis) +
      translation.dot(angular0.clone().cross(axis)),
    distance: translation.length(),
  };
}
