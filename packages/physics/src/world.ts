import type { Matrix4, Quaternion, Vector3, Object3D } from "three";
import { Joint } from "./joint.js";
import { RigidBody } from "./body.js";
import { authoredVelocity, setAuthoredVelocity } from "./velocity.js";
import { authoredJointReading } from "./reading.js";
import { setWorldPose } from "./transforms.js";
import type { Vec3, CollisionGroups } from "./objects.js";

export interface PhysicsOptions {
  readonly gravity?: Vec3;
  readonly fixedDelta?: number;
  readonly maxSubsteps?: number;
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
  readonly includeSensors?: boolean;
  readonly excludeBodies?: readonly RigidBody[];
}
export interface RaycastHit {
  distance: number;
  point: Vector3;
  normal: Vector3;
  body: RigidBody;
  collider: Object3D;
}

export interface PhysicsWorld {
  register(object: RigidBody | Joint): void;
  unregister(object: RigidBody | Joint): void;
  readonly fixedDelta: number;
  readonly time: number;
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null;
  update(delta: number): void;
  reset(): void;
  dispose(): void;
  /** Backend integration; validated scene commands arrive through body and joint methods. */
  getVelocity(object: RigidBody): PhysicsVelocity;
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void;
  teleport(object: RigidBody, matrix: Matrix4): void;
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void;
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void;
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void;
  wake(object: RigidBody): void;
  sleep(object: RigidBody): void;
  readJoint(object: Joint): JointReading;
  onBeforeStep(callback: (delta: number) => void): () => void;
  onAfterStep(callback: (delta: number) => void): () => void;
}

let defaultWorld: PhysicsWorld | undefined;

export function setDefaultWorld(world: PhysicsWorld): void {
  defaultWorld = world;
}

export function getDefaultWorld(): PhysicsWorld {
  if (!defaultWorld)
    throw new Error("Call setupWorld() before creating physics objects");
  return defaultWorld;
}

export function clearDefaultWorld(world: PhysicsWorld): void {
  if (defaultWorld === world) defaultWorld = undefined;
}

/** Owns authoring objects without loading a simulation backend. */
export class AuthoringWorld implements PhysicsWorld {
  readonly fixedDelta = 1 / 60;
  readonly time = 0;
  raycast(
    _origin: Vector3,
    _direction: Vector3,
    _maxDistance: number,
    _options?: RaycastOptions,
  ): never {
    throw new Error("AuthoringWorld does not support raycast queries");
  }
  private readonly registered = new Set<RigidBody | Joint>();
  private isDisposed = false;

  get objects(): ReadonlySet<RigidBody | Joint> {
    return this.registered;
  }

  register(object: RigidBody | Joint): void {
    if (this.isDisposed) throw new Error("Physics world has been disposed");
    if (object.world !== this)
      throw new Error("Object belongs to another physics world");
    if (object.disposed)
      throw new Error("Cannot register a disposed physics object");
    this.registered.add(object);
  }

  unregister(object: RigidBody | Joint): void {
    if (!this.registered.delete(object)) return;
    if (object instanceof RigidBody) {
      for (const joint of this.registered) {
        if (
          joint instanceof Joint &&
          (joint.options.body0 === object || joint.options.body1 === object)
        )
          joint.dispose();
      }
    }
  }

  dispose(): void {
    if (this.isDisposed) return;
    for (const object of this.registered) object.dispose();
    this.isDisposed = true;
    clearDefaultWorld(this);
  }

  update(_delta: number): never {
    return this.unavailable();
  }
  reset(): never {
    return this.unavailable();
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    this.assertObject(object);
    return authoredVelocity(object);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertObject(object);
    setAuthoredVelocity(object, value);
  }
  teleport(object: RigidBody, matrix: Matrix4): void {
    this.assertObject(object);
    object.validate();
    setWorldPose(object, matrix);
  }
  // Commands are inert by contract: a static preview accepts them without simulating (see README).
  setKinematicTarget(object: RigidBody, _matrix: Matrix4): void {
    this.assertObject(object);
  }
  applyImpulse(object: RigidBody, _impulse: Vector3, _point?: Vector3): void {
    this.assertObject(object);
  }
  applyForce(object: RigidBody, _force: Vector3, _point?: Vector3): void {
    this.assertObject(object);
  }
  wake(object: RigidBody): void {
    this.assertObject(object);
  }
  sleep(object: RigidBody): void {
    this.assertObject(object);
  }
  readJoint(object: Joint): JointReading {
    this.assertObject(object);
    return authoredJointReading(object);
  }
  onBeforeStep(_callback: (delta: number) => void): () => void {
    if (this.isDisposed) throw new Error("Physics world has been disposed");
    return () => {};
  }
  onAfterStep(_callback: (delta: number) => void): () => void {
    return this.onBeforeStep(_callback);
  }
  private assertObject(object: RigidBody | Joint): void {
    if (this.isDisposed) throw new Error("Physics world has been disposed");
    if (object.disposed) throw new Error("Physics object has been disposed");
    if (object.world !== this)
      throw new Error("Object belongs to another world");
  }

  private unavailable(): never {
    throw new Error(
      "AuthoringWorld cannot simulate; use a simulation backend's setupWorld()",
    );
  }
}
