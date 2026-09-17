import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import { AxisJoint, DistanceJoint, PrismaticJoint } from "./joints.js";
import { GenericJoint } from "./generic.js";
import type { Joint } from "./joint.js";
import { axisVector, splitTransform } from "./transforms.js";
import { authoredVelocityAtPoint } from "./velocity.js";
import type { JointReading } from "./world.js";

export function authoredJointReading(
  object: Joint,
  frames?: readonly [Matrix4, Matrix4],
  velocityAtPoint: (
    body: RigidBody,
    point: Vector3,
  ) => Vector3 = authoredVelocityAtPoint,
): JointReading {
  object.validate();
  const options = object.options;
  const frame = (index: 0 | 1): Matrix4 => {
    const body = index === 0 ? options.body0 : options.body1;
    const matrix = frames
      ? frames[index].clone()
      : object.getFrame(index, new Matrix4());
    if (body) matrix.premultiply(splitTransform(body.matrixWorld).pose);
    return matrix;
  };
  const a = frame(0),
    b = frame(1);
  if (object instanceof AxisJoint) {
    const rotation = new Matrix4().makeRotationFromQuaternion(
      new Quaternion().setFromUnitVectors(
        new Vector3(1, 0, 0),
        axisVector(object.options.axis),
      ),
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
  // Only translational readings need anchor velocities, which authoring infers from explicit mass properties.
  const translational =
    object instanceof PrismaticJoint ||
    object instanceof DistanceJoint ||
    object instanceof GenericJoint;
  const anchor0 = new Vector3().setFromMatrixPosition(a);
  const anchor1 = new Vector3().setFromMatrixPosition(b);
  const linear0 =
    translational && body0 ? velocityAtPoint(body0, anchor0) : new Vector3();
  const linear1 = translational
    ? velocityAtPoint(body1, anchor1)
    : new Vector3();
  return jointReading(
    a,
    b,
    velocity0.angular,
    velocity1.angular,
    linear0,
    linear1,
  );
}

/** Frame-0 reading from world-frame inputs, used by simulation adapters. The angle is wrapped to [-pi, pi]. */
export function jointReading(
  a: Matrix4,
  b: Matrix4,
  angular0: Vector3,
  angular1: Vector3,
  linear0 = new Vector3(),
  linear1 = new Vector3(),
): JointReading {
  const inverse0 = new Quaternion().setFromRotationMatrix(a).invert();
  const rotation = inverse0
    .clone()
    .multiply(new Quaternion().setFromRotationMatrix(b))
    .normalize();
  const translation = new Vector3()
    .setFromMatrixPosition(b)
    .sub(new Vector3().setFromMatrixPosition(a));
  // Anchor 1 as seen from frame 0, which rotates with body 0.
  const linear = linear1
    .clone()
    .sub(linear0)
    .sub(angular0.clone().cross(translation));
  const angle = 2 * Math.atan2(rotation.x, rotation.w);
  return {
    translation: translation.applyQuaternion(inverse0),
    rotation,
    linearVelocity: linear.applyQuaternion(inverse0),
    angularVelocity: angular1.clone().sub(angular0).applyQuaternion(inverse0),
    angle: wrapAngle(angle),
  };
}

/** Wraps an angle into [-π, π]. */
export function wrapAngle(angle: number): number {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}
