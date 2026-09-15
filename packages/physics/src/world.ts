import type { Matrix4, Vector3, Quaternion, Object3D } from "three";
import { Joint, AxisJoint } from "./joints.js";
import { RigidBody } from "./body.js";
import {
  authoredVelocity,
  setAuthoredVelocity,
  setWorldPose,
  authoredJointState,
} from "./state.js";
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
export interface FixedJointState {
  translation: Vector3;
  rotation: Quaternion;
}
export interface SphericalJointState {
  distance: number;
}
export interface DistanceJointState {
  distance: number;
}
export type PhysicsJointState =
  AxisJointState | FixedJointState | SphericalJointState | DistanceJointState;
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
  setJointEffort(object: AxisJoint, value: number): void;
  update(delta: number): void;
  reset(): void;
  dispose(): void;
  /** Backend integration; scene code calls methods on bodies and joints. */
  getVelocity(object: RigidBody): PhysicsVelocity;
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void;
  teleport(object: RigidBody, matrix: Matrix4): void;
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void;
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void;
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void;
  wake(object: RigidBody): void;
  sleep(object: RigidBody): void;
  getJointState(object: Joint): PhysicsJointState;
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
  get fixedDelta(): number {
    return 1 / 60;
  }
  get time(): number {
    return 0;
  }
  raycast(
    _origin: Vector3,
    _direction: Vector3,
    _maxDistance: number,
    _options?: RaycastOptions,
  ): never {
    throw new Error("AuthoringWorld does not support raycast queries");
  }
  setJointEffort(object: AxisJoint, value: number): void {
    this.assertObject(object);
    if (!Number.isFinite(value)) throw new Error("Joint effort must be finite");
  }
  readonly #objects = new Set<RigidBody | Joint>();
  #disposed = false;

  get objects(): ReadonlySet<RigidBody | Joint> {
    return this.#objects;
  }

  register(object: RigidBody | Joint): void {
    if (this.#disposed) throw new Error("Physics world has been disposed");
    if (object.world !== this)
      throw new Error("Object belongs to another physics world");
    if (object.disposed)
      throw new Error("Cannot register a disposed physics object");
    this.#objects.add(object);
  }

  unregister(object: RigidBody | Joint): void {
    if (!this.#objects.delete(object)) return;
    if (object instanceof RigidBody) {
      for (const joint of this.#objects) {
        if (
          joint instanceof Joint &&
          (joint.options.body0 === object || joint.options.body1 === object)
        )
          joint.dispose();
      }
    }
  }

  dispose(): void {
    if (this.#disposed) return;
    for (const object of this.#objects) object.dispose();
    this.#disposed = true;
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
  getJointState(object: Joint): PhysicsJointState {
    this.assertObject(object);
    return authoredJointState(object);
  }
  onBeforeStep(_callback: (delta: number) => void): () => void {
    if (this.#disposed) throw new Error("Physics world has been disposed");
    return () => {};
  }
  onAfterStep(_callback: (delta: number) => void): () => void {
    return this.onBeforeStep(_callback);
  }
  private assertObject(object: RigidBody | Joint): void {
    if (this.#disposed) throw new Error("Physics world has been disposed");
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
