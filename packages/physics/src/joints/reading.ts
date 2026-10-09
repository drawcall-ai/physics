import { Matrix4, Quaternion, Vector3 } from "three";
import type { RigidBody } from "../body.js";
import { AxisJoint } from "./kinds.js";
import type { Joint } from "./joint.js";
import { axisVector, splitTransform } from "../transforms.js";

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
/** Frame 1 relative to frame 0, in frame 0 coordinates. */
export interface JointPose {
  translation: Vector3;
  rotation: Quaternion;
}
/**
 * Backend integration: the joint's pose, and velocities relative to frame 0 as a moving frame.
 * Typed joint states derive from this one reading.
 */
export interface JointReading extends JointPose {
  /** Anchor 1 relative to anchor 0. */
  linearVelocity: Vector3;
  /** Body 1 relative to body 0. */
  angularVelocity: Vector3;
  /** Rotation about the frame X axis; backends report it continuously across turns. */
  angle: number;
}

/** Where a joint reading takes its bodies' motion from; each call returns a new vector. */
export interface Motion {
  angular(body: RigidBody): Vector3;
  velocityAt(body: RigidBody, point: Vector3): Vector3;
}

/** Frame `b` relative to frame `a`, in `a` coordinates. */
function relativePose(a: Matrix4, b: Matrix4): JointPose {
  const inverse = new Quaternion().setFromRotationMatrix(a).invert();
  return {
    translation: new Vector3()
      .setFromMatrixPosition(b)
      .sub(new Vector3().setFromMatrixPosition(a))
      .applyQuaternion(inverse),
    rotation: inverse
      .clone()
      .multiply(new Quaternion().setFromRotationMatrix(b))
      .normalize(),
  };
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
    const { body0, body1 } = this.joint.options;
    const [a, b] = this.placedFrames();
    const anchor0 = new Vector3().setFromMatrixPosition(a);
    const anchor1 = new Vector3().setFromMatrixPosition(b);
    const angular0 = body0 ? motion.angular(body0) : new Vector3();
    const linear0 = body0 ? motion.velocityAt(body0, anchor0) : new Vector3();
    // Anchor 1 as seen from frame 0, which rotates with body 0.
    const linear = motion
      .velocityAt(body1, anchor1)
      .sub(linear0)
      .sub(angular0.clone().cross(anchor1.sub(anchor0)));
    const inverse0 = new Quaternion().setFromRotationMatrix(a).invert();
    return {
      ...relativePose(a, b),
      linearVelocity: linear.applyQuaternion(inverse0),
      angularVelocity: motion
        .angular(body1)
        .sub(angular0)
        .applyQuaternion(inverse0),
      angle: this.continuous,
    };
  }
  /** The joint's pose from the scene's poses of its bodies. */
  pose(): JointPose {
    const [a, b] = this.placedFrames();
    return relativePose(a, b);
  }
  /** Restarts turn counting from the current angle, as creation, teleport and reset do. */
  rebase(): void {
    this.continuous = this.sampled = this.measure();
  }
  /** Restarts turn counting if a teleport moved one of the joint's bodies but not the other. */
  rebaseIfSplit(moved: ReadonlySet<RigidBody>): void {
    const { body0, body1 } = this.joint.options;
    if ((body0 !== null && moved.has(body0)) !== moved.has(body1))
      this.rebase();
  }
  /** Counts the turns since the last sample; motion must stay below π per sample. */
  track(): void {
    const angle = this.measure();
    this.continuous += wrapAngle(angle - this.sampled);
    this.sampled = angle;
  }
  /** The wrapped angle about the frame X axis the scene's poses give the joint now. */
  private measure(): number {
    const { x, w } = this.pose().rotation;
    return wrapAngle(2 * Math.atan2(x, w));
  }
  /** The captured frames placed on their bodies' poses in the scene, X along an axis joint's axis. */
  private placedFrames(): [Matrix4, Matrix4] {
    this.joint.validate();
    const { body0, body1 } = this.joint.options;
    const place = (frame: Matrix4, body: RigidBody | null): Matrix4 => {
      const matrix = frame.clone();
      if (body) matrix.premultiply(splitTransform(body.matrixWorld).pose);
      return matrix;
    };
    const a = place(this.frames[0], body0);
    const b = place(this.frames[1], body1);
    if (this.joint instanceof AxisJoint) {
      const rotation = new Matrix4().makeRotationFromQuaternion(
        new Quaternion().setFromUnitVectors(
          new Vector3(1, 0, 0),
          axisVector(this.joint.options.axis),
        ),
      );
      a.multiply(rotation);
      b.multiply(rotation);
    }
    return [a, b];
  }
}
