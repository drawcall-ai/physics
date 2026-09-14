import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import { AxisJoint, FixedJoint, type Joint } from "./joints.js";
import { splitTransform } from "./transforms.js";
import type { PhysicsVelocity, PhysicsJointState } from "./world.js";

/** Shared state operations for backend adapters and static authoring. */
export function initialVelocity(object: RigidBody): PhysicsVelocity {
  const linear = new Vector3(...(object.options.linearVelocity ?? [0, 0, 0]));
  const angular = new Vector3(...(object.options.angularVelocity ?? [0, 0, 0]));
  validateVector(linear);
  validateVector(angular);
  return { linear, angular };
}
export function setInitialVelocity(
  object: RigidBody,
  value: Partial<PhysicsVelocity>,
): void {
  if (value.linear)
    object.options.linearVelocity = [
      value.linear.x,
      value.linear.y,
      value.linear.z,
    ];
  if (value.angular)
    object.options.angularVelocity = [
      value.angular.x,
      value.angular.y,
      value.angular.z,
    ];
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
): PhysicsJointState {
  object.validate();
  const frame = (index: 0 | 1): Matrix4 => {
    const body = index === 0 ? object.options.body0 : object.options.body1;
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
  return jointState(
    a,
    b,
    object.options.body0?.getVelocity().angular ?? new Vector3(),
    object.options.body1.getVelocity().angular,
  );
}

export function jointState(
  a: Matrix4,
  b: Matrix4,
  angular0: Vector3,
  angular1: Vector3,
): PhysicsJointState {
  const rotation = new Quaternion().setFromRotationMatrix(a);
  const relative = rotation
    .clone()
    .invert()
    .multiply(new Quaternion().setFromRotationMatrix(b));
  const axis = new Vector3(1, 0, 0).applyQuaternion(rotation);
  const position0 = new Vector3().setFromMatrixPosition(a);
  const position1 = new Vector3().setFromMatrixPosition(b);
  return {
    angle: 2 * Math.atan2(relative.x, relative.w),
    angularVelocity: angular1.clone().sub(angular0).dot(axis),
    position: position1.clone().sub(position0).dot(axis),
    distance: position0.distanceTo(position1),
  };
}
