import type { Matrix4, Vector3, Object3D } from "three";
import { RigidBody } from "./body.js";
import { Joint } from "./joints/joint.js";
import type { JointReading } from "./joints/reading.js";
import { Trigger } from "./trigger.js";
import type { Vec3 } from "./colliders/collider.js";
import type { RaycastHit, RaycastOptions } from "./raycast.js";
import { cleanup, rollback } from "./cleanup.js";
import { Interactions } from "./interactions.js";

export interface PhysicsWorldOptions {
  /** The world simulates the bodies, joints and triggers under this object, normally the three.js Scene. */
  readonly scene: Object3D;
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

/**
 * The fixed-step clock, step callbacks, event dispatch and lifecycle every backend shares.
 * Before every step and query, the world syncs its members with the objects under its scene and
 * prepares them, so every member has live backend state. A backend supplies the simulation: how
 * objects enter and leave it, how authored changes reach it, the step itself, how its state is
 * restored and freed, and the member commands that bodies and joints forward once validated.
 */
export abstract class PhysicsWorld {
  /** The world simulates the bodies, joints and triggers under this object. */
  readonly scene: Object3D;
  readonly fixedDelta: number;
  protected readonly gravity: Vec3;
  /** Undefined leaves the backend's own default in place. */
  protected readonly solverIterations: number | undefined;
  protected readonly interactions = new Interactions();
  private readonly maxSubsteps: number;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  private readonly members = new Set<RigidBody | Joint | Trigger>();
  private elapsed = 0;
  private completed = 0;
  /** Inside an update or refresh; a world disposed meanwhile is freed when it ends. */
  private busy = false;
  private isDisposed = false;

  constructor(options: PhysicsWorldOptions) {
    this.scene = options.scene;
    this.fixedDelta = options.fixedDelta ?? 1 / 60;
    this.maxSubsteps = options.maxSubsteps ?? 5;
    this.gravity = options.gravity ?? [0, -9.81, 0];
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

  /** Adds an object that entered the scene; a joint arrives after its bodies. */
  protected abstract add(object: RigidBody | Joint | Trigger): void;
  /** Removes an object that left the scene; a joint leaves before its bodies. */
  protected abstract remove(object: RigidBody | Joint | Trigger): void;
  /** Brings the backend up to date with the members and their authored changes. */
  protected abstract prepare(): void;
  /** Advances the prepared simulation one fixed step, synchronizes the scene and samples interactions. */
  protected abstract step(): void;
  /** Returns the members to the state captured when they were first prepared. */
  protected abstract restore(): void;
  /** Releases backend memory, once, after disposal and outside any update or refresh. */
  protected abstract free(): void;
  /** Raycasts against the prepared members. */
  protected abstract cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null;

  // Member commands: bodies and joints validate a command, then forward it to their world here.
  abstract getVelocity(object: RigidBody): PhysicsVelocity;
  abstract setVelocity(
    object: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void;
  /**
   * Adopts the poses the scene already holds for the body and the dynamic bodies jointed to it.
   * An assembly articulated to a static or kinematic base or the world can only move within
   * those joints; a backend that cannot honour that rejects the call.
   */
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

  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null {
    this.assertLive();
    this.refresh();
    this.assertLive();
    for (const body of options?.excludeBodies ?? []) this.assertMember(body);
    return this.cast(origin, direction, maxDistance, options);
  }
  getOverlappingBodies(trigger: Trigger): RigidBody[] {
    this.assertLive();
    this.refresh();
    this.assertMember(trigger);
    return this.interactions.bodies(trigger);
  }
  update(delta: number): void {
    this.assertIdle();
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative");
    this.run(() => {
      // A failed update adds no time; only the steps it completed advance the clock.
      let elapsed = Math.min(
        this.elapsed + delta,
        this.fixedDelta * this.maxSubsteps,
      );
      if (elapsed < this.fixedDelta) this.refresh();
      while (!this.isDisposed && elapsed >= this.fixedDelta) {
        // Before-step callbacks see the objects that joined since the last step, and the step
        // sees the objects those callbacks added.
        if (this.before.size) this.refresh();
        if (this.isDisposed) return;
        for (const callback of this.before) {
          callback(this.fixedDelta);
          if (this.isDisposed) return;
        }
        this.refresh();
        if (this.isDisposed) return;
        this.step();
        elapsed -= this.fixedDelta;
        this.completed += this.fixedDelta;
        this.interactions.dispatch();
        for (const callback of this.after) {
          if (this.isDisposed) return;
          callback(this.fixedDelta);
        }
      }
      this.elapsed = elapsed;
    });
  }
  reset(): void {
    this.assertIdle();
    this.interactions.clear();
    this.restore();
    this.elapsed = 0;
    this.completed = 0;
    this.prepare();
  }
  dispose(): void {
    if (this.isDisposed) return;
    this.detach([...this.members]);
    this.isDisposed = true;
    this.interactions.clear();
    this.members.clear();
    this.before.clear();
    this.after.clear();
    if (!this.busy) this.free();
  }
  onBeforeStep(callback: (delta: number) => void): () => void {
    this.assertLive();
    this.before.add(callback);
    return () => {
      this.before.delete(callback);
    };
  }
  onAfterStep(callback: (delta: number) => void): () => void {
    this.assertLive();
    this.after.add(callback);
    return () => {
      this.after.delete(callback);
    };
  }
  /** Throws unless this live world simulates `object`. */
  protected assertMember(object: RigidBody | Joint | Trigger): void {
    this.assertLive();
    if (object.world !== this)
      throw new Error("Physics object is outside the world's scene");
  }
  private assertLive(): void {
    if (this.isDisposed) throw new Error("Physics world has been disposed");
  }
  private assertIdle(): void {
    this.assertLive();
    if (this.busy)
      throw new Error(
        "Cannot update or reset during a physics step or event dispatch",
      );
  }
  /** Runs `work`; the outermost run frees the world if a callback disposed it meanwhile. */
  private run(work: () => void): void {
    if (this.busy) return work();
    this.busy = true;
    try {
      work();
    } finally {
      this.busy = false;
      if (this.isDisposed) this.free();
    }
  }
  /**
   * Syncs and prepares the members, then delivers the events their removal queued. Listeners
   * may dispose the world, so callers check before going on. Joining is all or nothing: if
   * preparing fails, the objects that were joining leave again and join afresh next time.
   */
  private refresh(): void {
    this.run(() => {
      const joining: (RigidBody | Joint | Trigger)[] = [];
      try {
        for (const object of this.sync()) {
          if (object.world)
            throw new Error(
              "Physics object is already simulated by another world; remove it from that world's scene and update or dispose that world first",
            );
          this.add(object);
          this.members.add(object);
          joining.push(object);
        }
        this.prepare();
      } catch (error) {
        rollback(
          error,
          joining.reverse().map((object) => () => {
            this.members.delete(object);
            this.remove(object);
          }),
          "Physics object join failed",
        );
      }
      for (const object of this.members) object.world = this;
      this.interactions.dispatch();
    });
  }
  /** Releases objects from this world; dynamic bodies keep their simulated velocity. */
  private detach(objects: readonly (RigidBody | Joint | Trigger)[]): void {
    const velocities = objects
      .filter(
        (object): object is RigidBody =>
          object instanceof RigidBody &&
          object.bodyType === "dynamic" &&
          object.world === this,
      )
      .map((body) => [body, this.getVelocity(body)] as const);
    for (const object of objects) object.world = undefined;
    for (const [body, velocity] of velocities) body.setVelocity(velocity);
  }
  /** Removes the objects that left the scene; returns those that entered it, joints last. */
  private sync(): (RigidBody | Joint | Trigger)[] {
    const current = new Set<RigidBody | Joint | Trigger>();
    const joints: Joint[] = [];
    this.scene.traverse((object) => {
      if (object instanceof Joint) joints.push(object);
      else if (object instanceof RigidBody || object instanceof Trigger)
        current.add(object);
    });
    for (const joint of joints) {
      const { body0, body1 } = joint.options;
      if ((body0 && !current.has(body0)) || !current.has(body1))
        throw new Error("Joint connects a body outside the world's scene");
      current.add(joint);
    }
    const leaving = [...this.members]
      .filter((object) => !current.has(object))
      .sort((a, b) => Number(b instanceof Joint) - Number(a instanceof Joint));
    this.detach(leaving);
    cleanup(
      leaving.map((object) => () => {
        this.members.delete(object);
        if (!(object instanceof Joint)) this.interactions.remove(object);
        this.remove(object);
      }),
      "Physics object removal failed",
    );
    return [...current].filter((object) => !this.members.has(object));
  }
}

/** Brings a new world up to date with its scene, disposing it if that fails. */
export function build<World extends PhysicsWorld>(world: World): World {
  try {
    world.update(0);
  } catch (error) {
    rollback(error, [() => world.dispose()], "Physics world build failed");
  }
  return world;
}

/** The world simulating `object`; simulation commands need one. */
export function requireWorld(
  object: RigidBody | Joint | Trigger,
): PhysicsWorld {
  if (!object.world)
    throw new Error(
      "Physics object has not joined a world yet; it joins at the world's next update or query (world.update(0) joins it now)",
    );
  return object.world;
}
