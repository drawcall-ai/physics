import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import {
  AxisJoint,
  PrismaticJoint,
  RevoluteJoint,
  FixedJoint,
  type Joint,
} from "./joints.js";
import { splitTransform } from "./transforms.js";
import { centerOfMass } from "./center.js";
import type { PhysicsVelocity, PhysicsJointState } from "./world.js";

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
  if (value.linear) {
    validateVector(value.linear);
    next.linear.copy(value.linear);
  }
  if (value.angular) {
    validateVector(value.angular);
    next.angular.copy(value.angular);
  }
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
  centers?: readonly [Vector3, Vector3],
): PhysicsJointState {
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
  const origin0 =
    centers?.[0] ??
    (object instanceof PrismaticJoint &&
    body0 &&
    velocity0.angular.lengthSq() > 0
      ? centerOfMass(body0)
      : new Vector3());
  const origin1 =
    centers?.[1] ??
    (object instanceof PrismaticJoint && velocity1.angular.lengthSq() > 0
      ? centerOfMass(body1)
      : new Vector3());
  const linear0 = velocity0.linear
    .clone()
    .add(velocity0.angular.clone().cross(anchor0.clone().sub(origin0)));
  const linear1 = velocity1.linear
    .clone()
    .add(velocity1.angular.clone().cross(anchor1.clone().sub(origin1)));
  const measured = jointState(
    a,
    b,
    velocity0.angular,
    velocity1.angular,
    linear0,
    linear1,
  );
  if (object instanceof RevoluteJoint)
    return { position: measured.angle, velocity: measured.angularVelocity };
  if (object instanceof AxisJoint)
    return { position: measured.position, velocity: measured.velocity };
  if (object instanceof FixedJoint)
    return { translation: measured.translation, rotation: measured.rotation };
  return { distance: measured.distance };
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
    translation: translation.clone().applyQuaternion(rotation.clone().invert()),
    rotation: relative,
  };
}
