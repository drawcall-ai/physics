import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "../body.js";
import { AxisJoint, DistanceJoint, PrismaticJoint } from "./kinds.js";
import { GenericJoint } from "./generic.js";
import type { Joint } from "./joint.js";
import { axisVector, splitTransform } from "../transforms.js";
import type { PhysicsVelocity } from "../world.js";

export interface AxisJointState {
  position: number;
  velocity: number;
}
export interface SphericalJointState {
  /** Frame 1 relative to frame 0. */
  rotation: Quaternion;
  /** Body 1 relative to body 0, in frame 0 coordinates. */
  angularVelocity: Vector3;
}
export interface DistanceJointState {
  distance: number;
  velocity: number;
}
/**
 * Backend integration: frame 1 relative to frame 0, in frame 0 coordinates. Velocities are relative
 * to frame 0 as a moving frame. Typed joint states derive from this one reading.
 */
export interface JointReading {
  translation: Vector3;
  rotation: Quaternion;
  /** Anchor 1 relative to anchor 0. */
  linearVelocity: Vector3;
  /** Body 1 relative to body 0. */
  angularVelocity: Vector3;
  /** Rotation about the frame X axis; backends report it continuously across turns. */
  angle: number;
}

/** Where a joint reading takes its bodies' motion from. */
export interface Motion {
  velocity(body: RigidBody): PhysicsVelocity;
  velocityAt(body: RigidBody, point: Vector3): Vector3;
}

/** Motion for readings that only need poses. */
export const still: Motion = {
  velocity: () => ({ linear: new Vector3(), angular: new Vector3() }),
  velocityAt: () => new Vector3(),
};

/**
 * The joint's reading from the scene graph as it stands: body poses from their world matrices
 * and velocities from the bodies. Backends pass the frames they captured and the motion they
 * measure.
 */
export function authoredJointReading(
  object: Joint,
  frames?: readonly [Matrix4, Matrix4],
  motion?: Motion,
): JointReading {
  object.validate();
  const { body0, body1 } = object.options;
  const frame = (index: 0 | 1): Matrix4 => {
    const body = index === 0 ? body0 : body1;
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
  const velocityOf = (body: RigidBody) =>
    motion ? motion.velocity(body) : body.getVelocity();
  const velocity0 = body0 ? velocityOf(body0) : still.velocity(body1);
  const velocity1 = velocityOf(body1);
  // Without a backend measurement, only translational readings need anchor velocities, which
  // authoring can infer solely from explicit mass properties.
  const anchors =
    motion !== undefined ||
    object instanceof PrismaticJoint ||
    object instanceof DistanceJoint ||
    object instanceof GenericJoint;
  const velocityAt = motion?.velocityAt ?? velocityAtPoint;
  const anchor0 = new Vector3().setFromMatrixPosition(a);
  const anchor1 = new Vector3().setFromMatrixPosition(b);
  const linear0 = anchors && body0 ? velocityAt(body0, anchor0) : new Vector3();
  const linear1 = anchors ? velocityAt(body1, anchor1) : new Vector3();
  return jointReading(
    a,
    b,
    velocity0.angular,
    velocity1.angular,
    linear0,
    linear1,
  );
}

/** Frame-0 reading from world-frame inputs. The angle is wrapped to [-pi, pi]. */
function jointReading(
  a: Matrix4,
  b: Matrix4,
  angular0: Vector3,
  angular1: Vector3,
  linear0: Vector3,
  linear1: Vector3,
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

/** The body's velocity at a world point, from its velocity and its explicit center of mass. */
function velocityAtPoint(body: RigidBody, point: Vector3): Vector3 {
  const { linear, angular } = body.getVelocity();
  if (angular.lengthSq() === 0) return linear;
  const center = body.options.centerOfMass;
  if (!center)
    throw new Error(
      "Authoring the anchor velocity of a rotating body requires explicit mass properties",
    );
  const worldCenter = new Vector3(...center).applyMatrix4(
    splitTransform(body.matrixWorld).pose,
  );
  return linear.add(angular.cross(point.clone().sub(worldCenter)));
}

/** Wraps an angle into [-π, π]. */
export function wrapAngle(angle: number): number {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

/**
 * A joint as a backend holds it: the frames captured as it joins, and the turns its angle has made
 * since, so the angle reads continuously.
 */
export class JointBinding {
  readonly frames: readonly [Matrix4, Matrix4];
  private continuous = 0;
  private sampled = 0;

  constructor(readonly joint: Joint) {
    this.frames = [
      joint.getFrame(0, new Matrix4()),
      joint.getFrame(1, new Matrix4()),
    ];
    this.rebase();
  }
  /** The continuous angle. */
  get angle(): number {
    return this.continuous;
  }
  /** The reading from the scene's poses and the backend's motion, with the continuous angle. */
  read(motion: Motion): JointReading {
    const reading = authoredJointReading(this.joint, this.frames, motion);
    return { ...reading, angle: this.continuous };
  }
  /** Restarts turn counting from the current angle, as creation, teleport and reset do. */
  rebase(): void {
    this.continuous = this.sampled = this.measure();
  }
  /** Counts the turns since the last sample; motion must stay below π per sample. */
  track(): void {
    const angle = this.measure();
    this.continuous += wrapAngle(angle - this.sampled);
    this.sampled = angle;
  }
  /** The wrapped angle the scene's poses give the joint now. */
  private measure(): number {
    return authoredJointReading(this.joint, this.frames, still).angle;
  }
}

export function readJoint(object: Joint): JointReading {
  return object.world?.readJoint(object) ?? authoredJointReading(object);
}
