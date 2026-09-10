import type { Matrix4, Vector3 } from "three";
import { Joint } from "./joints.js";
import { RigidBody } from "./body.js";
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

export interface PhysicsBodyControls {
  getMatrix(target?: Matrix4): Matrix4;
  getVelocity(): PhysicsVelocity;
  setVelocity(value: Partial<PhysicsVelocity>): void;
  setKinematicTarget(matrix: Matrix4): void;
  teleport(matrix: Matrix4): void;
  applyImpulse(impulse: Vector3, worldPoint?: Vector3): void;
  applyForce(force: Vector3, worldPoint?: Vector3): void;
  wake(): void;
  sleep(): void;
}

export interface PhysicsJointState {
  angle: number;
  angularVelocity: number;
  position: number;
  distance: number;
}

export interface PhysicsJointControls {
  getState(): PhysicsJointState;
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
  body(object: RigidBody): PhysicsBodyControls;
  joint(object: Joint): PhysicsJointControls;
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
  body(_object: RigidBody): never {
    return this.unavailable();
  }
  joint(_object: Joint): never {
    return this.unavailable();
  }
  onBeforeStep(_callback: (delta: number) => void): never {
    return this.unavailable();
  }
  onAfterStep(_callback: (delta: number) => void): never {
    return this.unavailable();
  }

  private unavailable(): never {
    throw new Error(
      "AuthoringWorld cannot simulate; use a simulation backend's setupWorld()",
    );
  }
}
