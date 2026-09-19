import type { Matrix4, Vector3 } from "three";
import type { RigidBody } from "./body.js";
import type { Joint } from "./joint.js";
import type { Trigger } from "./trigger.js";
import type { Vec3 } from "./colliders.js";
import { cleanup } from "./cleanup.js";
import { Interactions } from "./interactions.js";
import { registry, assertOwned } from "./registry.js";
import {
  assertLive,
  type JointReading,
  type PhysicsOptions,
  type PhysicsVelocity,
  type PhysicsWorld,
  type RaycastHit,
  type RaycastOptions,
} from "./world.js";

/**
 * The fixed-step clock, step callbacks, event dispatch and lifecycle every backend shares.
 * A backend supplies the simulation: what to reconcile before a step, the step itself, and how
 * its state is restored, released and freed.
 */
export abstract class SteppedWorld implements PhysicsWorld {
  readonly fixedDelta: number;
  protected readonly gravity: Vec3;
  /** Undefined leaves the backend's own default in place. */
  protected readonly solverIterations: number | undefined;
  protected readonly interactions = new Interactions();
  private readonly maxSubsteps: number;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  private elapsed = 0;
  private completed = 0;
  private updating = false;
  private isDisposed = false;

  constructor(options: PhysicsOptions) {
    this.fixedDelta = options.fixedDelta ?? 1 / 60;
    this.maxSubsteps = options.maxSubsteps ?? 5;
    this.gravity = [...(options.gravity ?? [0, -9.81, 0])];
    this.solverIterations = options.solverIterations;
    if (!Number.isFinite(this.fixedDelta) || this.fixedDelta <= 0)
      throw new Error("fixedDelta must be positive and finite");
    if (!Number.isInteger(this.maxSubsteps) || this.maxSubsteps < 1)
      throw new Error("maxSubsteps must be a positive integer");
    if (!this.gravity.every(Number.isFinite))
      throw new Error("Gravity must be finite");
    if (
      this.solverIterations !== undefined &&
      (!Number.isInteger(this.solverIterations) || this.solverIterations < 1)
    )
      throw new Error("solverIterations must be a positive integer");
  }
  get time(): number {
    return this.completed;
  }
  get disposed(): boolean {
    return this.isDisposed;
  }

  abstract register(object: RigidBody | Joint | Trigger): void;
  abstract unregister(object: RigidBody | Joint | Trigger): void;
  abstract raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null;
  abstract getVelocity(object: RigidBody): PhysicsVelocity;
  abstract setVelocity(
    object: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void;
  abstract teleport(object: RigidBody): void;
  abstract setKinematicTarget(object: RigidBody, matrix: Matrix4): void;
  abstract applyImpulse(
    object: RigidBody,
    impulse: Vector3,
    point?: Vector3,
  ): void;
  abstract applyForce(object: RigidBody, force: Vector3, point?: Vector3): void;
  abstract wake(object: RigidBody): void;
  abstract sleep(object: RigidBody): void;
  abstract readJoint(object: Joint): JointReading;
  /** Reconciles authored changes with the backend; runs on every update, before any step. */
  protected abstract prepare(): void;
  /** Reconciles again, advances the simulation one fixed step, synchronizes the scene and samples interactions. */
  protected abstract step(): void;
  /** Returns the backend to the state captured when its objects were first prepared. */
  protected abstract restore(): void;
  protected abstract disposeObjects(): void;
  /** Releases backend memory once the world is disposed and idle; may be called more than once. */
  protected abstract free(): void;

  update(delta: number): void {
    assertLive(this);
    this.assertIdle();
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative");
    this.updating = true;
    try {
      this.prepare();
      this.elapsed = Math.min(
        this.elapsed + delta,
        this.fixedDelta * this.maxSubsteps,
      );
      while (!this.isDisposed && this.elapsed >= this.fixedDelta) {
        for (const callback of this.before) {
          callback(this.fixedDelta);
          if (this.isDisposed) return;
        }
        this.step();
        this.elapsed -= this.fixedDelta;
        this.completed += this.fixedDelta;
        this.dispatch();
        for (const callback of this.after) {
          if (this.isDisposed) return;
          callback(this.fixedDelta);
        }
      }
    } finally {
      this.updating = false;
      if (this.isDisposed) this.free();
    }
  }
  reset(): void {
    assertLive(this);
    this.assertIdle();
    this.interactions.clear();
    this.restore();
    this.elapsed = 0;
    this.completed = 0;
  }
  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    this.interactions.clear();
    cleanup(
      [
        () => this.disposeObjects(),
        () => {
          this.before.clear();
          this.after.clear();
          registry.detach(this);
          if (!this.updating && !this.interactions.dispatching) this.free();
        },
      ],
      "Physics world disposal failed",
    );
  }
  getOverlappingBodies(trigger: Trigger): RigidBody[] {
    assertOwned(this, trigger);
    return this.interactions.bodies(trigger);
  }
  onBeforeStep(callback: (delta: number) => void): () => void {
    assertLive(this);
    this.before.add(callback);
    return () => {
      this.before.delete(callback);
    };
  }
  onAfterStep(callback: (delta: number) => void): () => void {
    assertLive(this);
    this.after.add(callback);
    return () => {
      this.after.delete(callback);
    };
  }
  protected assertIdle(): void {
    if (this.updating || this.interactions.dispatching)
      throw new Error(
        "Cannot update or reset during a physics step or event dispatch",
      );
  }
  /** Delivers queued interaction events, then whatever listeners deferred. */
  protected dispatch(): void {
    if (this.isDisposed || this.interactions.dispatching) return;
    cleanup(
      [() => this.interactions.dispatch(), () => this.settle()],
      "Physics event dispatch failed",
    );
  }
  /** Work that had to wait for a dispatch to finish: at least freeing a world a listener disposed. */
  protected settle(): void {
    if (this.isDisposed && !this.updating) this.free();
  }
}
