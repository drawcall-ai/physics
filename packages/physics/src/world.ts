import type { Matrix4, Quaternion, Vector3, Object3D } from "three";
import type { Joint } from "./joint.js";
import type { Trigger } from "./trigger.js";
import type { RigidBody } from "./body.js";
import type { Vec3, CollisionGroups } from "./colliders.js";

export interface PhysicsOptions {
  readonly gravity?: Vec3;
  readonly fixedDelta?: number;
  readonly maxSubsteps?: number;
  /** Constraint solver iterations per step; the backend's own default when omitted. */
  readonly solverIterations?: number;
}

export interface PhysicsVelocity {
  linear: Vector3;
  angular: Vector3;
}

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
export interface RaycastOptions {
  readonly collisionGroups?: CollisionGroups;
  readonly includeTriggers?: boolean;
  readonly excludeBodies?: readonly RigidBody[];
}
interface RaycastGeometry {
  distance: number;
  point: Vector3;
  normal: Vector3;
  collider: Object3D;
}
export type RaycastHit = RaycastGeometry &
  ({ kind: "body"; body: RigidBody } | { kind: "trigger"; trigger: Trigger });

export interface PhysicsWorld {
  readonly disposed: boolean;
  register(object: RigidBody | Joint | Trigger): void;
  unregister(object: RigidBody | Joint | Trigger): void;
  readonly fixedDelta: number;
  readonly time: number;
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null;
  getOverlappingBodies(trigger: Trigger): RigidBody[];
  update(delta: number): void;
  reset(): void;
  dispose(): void;
  /** Backend integration; validated scene commands arrive through body and joint methods. */
  getVelocity(object: RigidBody): PhysicsVelocity;
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void;
  /**
   * Adopts the poses the scene already holds for the body and the dynamic bodies jointed to it.
   * An assembly articulated to a static or kinematic base or the world can only move within
   * those joints; a backend that cannot honour that rejects the call.
   */
  teleport(object: RigidBody): void;
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void;
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void;
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void;
  wake(object: RigidBody): void;
  sleep(object: RigidBody): void;
  readJoint(object: Joint): JointReading;
  onBeforeStep(callback: (delta: number) => void): () => void;
  onAfterStep(callback: (delta: number) => void): () => void;
}

export function assertLive(world: PhysicsWorld): void {
  if (world.disposed) throw new Error("Physics world has been disposed");
}
