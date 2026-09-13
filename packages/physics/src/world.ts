import type { Matrix4, Vector3 } from "three";
import { Joint } from "./joints.js";
import { RigidBody } from "./body.js";
import {
  initialVelocity,
  setInitialVelocity,
  setWorldPose,
  authoredJointState,
} from "./state.js";
import type { Vec3 } from "./objects.js";

export interface PhysicsOptions {
  gravity?: Vec3;
  fixedDelta?: number;
  maxSubsteps?: number;
}

export interface PhysicsVelocity {
  linear: Vector3;
  angular: Vector3;
}

export interface PhysicsJointState {
  angle: number;
  angularVelocity: number;
  position: number;
  distance: number;
}

export interface PhysicsWorld {
  register(object: RigidBody | Joint): void;
  unregister(object: RigidBody | Joint): void;
  readonly fixedDelta: number;
  readonly maxSubsteps: number;
  update(delta: number): void;
  step(): void;
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
  readonly fixedDelta = 1 / 60;
  readonly maxSubsteps = 5;
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
  step(): never {
    return this.unavailable();
  }
  reset(): never {
    return this.unavailable();
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    this.assertObject(object);
    return initialVelocity(object);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertObject(object);
    setInitialVelocity(object, value);
  }
  teleport(object: RigidBody, matrix: Matrix4): void {
    this.assertObject(object);
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
