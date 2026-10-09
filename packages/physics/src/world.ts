import type { BufferGeometry, Matrix4, Object3D, Vector3 } from "three";
import {
  RigidBody,
  assertBodyType,
  keepVelocity,
  validateVelocity,
} from "./body.js";
import { Joint } from "./joints/joint.js";
import { authoredJointReading, type JointReading } from "./joints/reading.js";
import { placeAssembly } from "./joints/assembly.js";
import type { Trigger } from "./trigger.js";
import type { Vec3 } from "./colliders/collider.js";
import { prepareConvexParts } from "./colliders/decomposition.js";
import {
  rayDirection,
  type RaycastHit,
  type RaycastOptions,
} from "./raycast.js";
import { assertRigidTransform, validateVector } from "./transforms.js";
import { cleanup, rollback } from "./cleanup.js";
import { Interactions } from "./interactions.js";
import {
  claim,
  claimScene,
  holds,
  releaseScene,
  scan,
  sceneWorld,
  setJoined,
  type Member,
} from "./membership.js";

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

/** Selects the triangle meshes a backend collides as convex parts rather than as triangles. */
export type Decomposes = (body: RigidBody, geometry: BufferGeometry) => boolean;

/** A body's pose and velocity as it joined, which `reset()` returns it to. */
export interface Initial {
  readonly pose: Matrix4;
  readonly velocity: PhysicsVelocity;
}

/** The live reading of a joint under a world's scene, or else the authored one; for joints. */
export let readJoint: (joint: Joint) => JointReading;
/** Syncs a world's members with its scene, as before a query; for membership. */
export let refresh: (world: PhysicsWorld) => void;

/**
 * The fixed-step clock, step callbacks, event dispatch and command validation every backend
 * shares. Before every step and query, the world syncs its membership with the objects under its
 * scene and prepares the members, so every member has live backend state.
 *
 * A backend implements the protected hooks below. The base class calls them only for joined
 * members and with validated arguments: finite vectors, rigid poses, and the body type a command
 * needs. Scene objects never call hooks; their commands go through the public methods.
 */
export abstract class PhysicsWorld {
  /** The world simulates the bodies, joints and triggers under this object. */
  readonly scene: Object3D;
  readonly fixedDelta: number;
  readonly gravity: Vec3;
  /** Undefined leaves the backend's own default in place. */
  protected readonly solverIterations: number | undefined;
  protected readonly interactions = new Interactions();
  private readonly decomposes: Decomposes;
  private readonly maxSubsteps: number;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  /** The objects under the scene that have joined the world. */
  private readonly members = new Set<Member>();
  private elapsed = 0;
  private completed = 0;
  /** Inside an update or refresh; a world disposed meanwhile is freed when it ends. */
  private busy = false;
  private isDisposed = false;

  constructor(options: PhysicsWorldOptions, decomposes: Decomposes) {
    this.scene = options.scene;
    this.fixedDelta = options.fixedDelta ?? 1 / 60;
    this.maxSubsteps = options.maxSubsteps ?? 5;
    this.gravity = options.gravity ?? [0, -9.81, 0];
    this.solverIterations = options.solverIterations;
    this.decomposes = decomposes;
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
  static {
    refresh = (world) => world.refresh();
    readJoint = (joint) => {
      const world = sceneWorld(joint);
      if (!world) return authoredJointReading(joint);
      world.join(joint);
      return world.jointReading(joint);
    };
  }
  get time(): number {
    return this.completed;
  }
  get disposed(): boolean {
    return this.isDisposed;
  }

  /** Adds an object that entered the scene; a joint arrives after its bodies. */
  protected abstract add(object: Member): void;
  /** Removes an object that left the scene; a joint leaves before its bodies. */
  protected abstract remove(object: Member): void;
  /** Brings the backend up to date with the members and their authored changes. */
  protected abstract prepare(): void;
  /** Advances the prepared simulation one fixed step, synchronizes the scene and samples interactions. */
  protected abstract step(): void;
  /** Returns the members to the state captured when they were first prepared. */
  protected abstract restore(): void;
  /** Releases backend memory, once, after disposal and outside any update or refresh. */
  protected abstract free(): void;
  /** Raycasts against the prepared members; `direction` has unit length. */
  protected abstract cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options: RaycastOptions,
  ): RaycastHit | null;
  protected abstract readVelocity(body: RigidBody): PhysicsVelocity;
  /** Changes the given parts of a dynamic body's velocity. */
  protected abstract writeVelocity(
    body: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void;
  /**
   * Adopts the poses the scene now holds for a teleported body and the dynamic bodies jointed to
   * it. A backend that cannot move an assembly articulated to a static or kinematic base or the
   * world throws, and the scene poses are put back.
   */
  protected abstract writePoses(bodies: ReadonlySet<RigidBody>): void;
  /** Moves a kinematic body to `pose` over the next step. */
  protected abstract writeTarget(body: RigidBody, pose: Matrix4): void;
  protected abstract writeImpulse(
    body: RigidBody,
    impulse: Vector3,
    point?: Vector3,
  ): void;
  /** Applies a force over the next step. */
  protected abstract writeForce(
    body: RigidBody,
    force: Vector3,
    point?: Vector3,
  ): void;
  protected abstract writeSleeping(body: RigidBody, sleeping: boolean): void;
  protected abstract jointReading(joint: Joint): JointReading;

  getVelocity(body: RigidBody): PhysicsVelocity {
    this.join(body);
    return this.readVelocity(body);
  }
  setVelocity(body: RigidBody, value: Partial<PhysicsVelocity>): void {
    validateVelocity(body, value);
    this.join(body);
    this.writeVelocity(body, value);
  }
  /**
   * Moves the body, and every dynamic body the world's joints hold to it, rigidly to `pose`. An
   * assembly articulated to a static or kinematic base or the world can only move within those
   * joints; a backend that cannot honour that rejects the call.
   */
  teleport(body: RigidBody, pose: Matrix4): void {
    assertRigidTransform(pose);
    this.assertLive();
    // The assembly follows only joints that have joined, as the backend's does.
    this.refresh();
    this.assertMember(body);
    const joints = [...this.members].filter(
      (object) => object instanceof Joint,
    );
    const { moved, restores } = placeAssembly(body, pose, joints);
    try {
      this.writePoses(moved);
    } catch (error) {
      rollback(error, restores, "Teleport rollback failed");
    }
  }
  setKinematicTarget(body: RigidBody, pose: Matrix4): void {
    assertBodyType(body, "kinematic", "A kinematic target");
    assertRigidTransform(pose);
    this.join(body);
    this.writeTarget(body, pose);
  }
  applyImpulse(body: RigidBody, impulse: Vector3, point?: Vector3): void {
    validateVector(impulse);
    if (point) validateVector(point);
    assertBodyType(body, "dynamic", "An impulse");
    this.join(body);
    this.writeImpulse(body, impulse, point);
  }
  applyForce(body: RigidBody, force: Vector3, point?: Vector3): void {
    validateVector(force);
    if (point) validateVector(point);
    assertBodyType(body, "dynamic", "A force");
    this.join(body);
    this.writeForce(body, force, point);
  }
  wake(body: RigidBody): void {
    this.join(body);
    this.writeSleeping(body, false);
  }
  sleep(body: RigidBody): void {
    this.join(body);
    this.writeSleeping(body, true);
  }
  getOverlappingBodies(trigger: Trigger): RigidBody[] {
    this.assertLive();
    this.refresh();
    this.assertMember(trigger);
    return this.interactions.bodies(trigger);
  }
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options: RaycastOptions = {},
  ): RaycastHit | null {
    const unit = rayDirection(origin, direction, maxDistance, options);
    this.assertLive();
    this.refresh();
    return this.cast(origin, unit, maxDistance, options);
  }
  /**
   * Decomposes the triangle meshes under `root` that this backend collides as convex parts.
   * Building a world decomposes those under its scene; await this for meshes that join later or
   * whose geometry changed, before they join.
   */
  async decompose(root: Object3D): Promise<void> {
    this.assertLive();
    await prepareConvexParts(root, this.decomposes);
  }
  update(delta: number): void {
    this.assertIdle();
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative");
    this.run(() => {
      this.elapsed = Math.min(
        this.elapsed + delta,
        this.fixedDelta * this.maxSubsteps,
      );
      try {
        this.steps();
      } catch (error) {
        // A failed update drops its unspent time, so fixing the cause doesn't replay it at once.
        this.elapsed = 0;
        throw error;
      }
    });
  }
  private steps(): void {
    if (this.elapsed < this.fixedDelta) return this.refresh();
    while (!this.isDisposed && this.elapsed >= this.fixedDelta) {
      // Before-step callbacks see the objects that joined since the last step, and the step
      // sees the objects those callbacks added. Callbacks added meanwhile run from the next step.
      if (this.before.size) this.refresh();
      if (this.isDisposed) return;
      for (const callback of [...this.before]) {
        // A callback an earlier one removed no longer runs.
        if (this.before.has(callback)) callback(this.fixedDelta);
        if (this.isDisposed) return;
      }
      this.refresh();
      if (this.isDisposed) return;
      this.step();
      this.elapsed -= this.fixedDelta;
      this.completed += this.fixedDelta;
      this.interactions.dispatch();
      for (const callback of [...this.after]) {
        if (this.isDisposed) return;
        if (this.after.has(callback)) callback(this.fixedDelta);
      }
    }
  }
  reset(): void {
    this.assertIdle();
    this.refresh();
    if (this.isDisposed) return;
    this.interactions.clear();
    this.restore();
    this.elapsed = 0;
    this.completed = 0;
  }
  /** Frees the world; members leave it, dynamic bodies keeping their simulated velocity. */
  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    this.interactions.clear();
    this.before.clear();
    this.after.clear();
    releaseScene(this);
    cleanup(
      [
        ...[...this.members].map((object) => () => this.release(object)),
        () => {
          if (!this.busy) this.free();
        },
      ],
      "Physics world disposal failed",
    );
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
  /**
   * Syncs and prepares the members, then delivers the events leavers queued, even if joining
   * failed. Listeners may dispose the world, so callers check before going on.
   */
  private refresh(): void {
    this.run(() =>
      cleanup(
        [() => this.sync(), () => this.interactions.dispatch()],
        "Physics world refresh failed",
      ),
    );
  }
  /**
   * Removes the members that left the scene and adds those that entered it. Joining is all or
   * nothing: if preparing fails, the objects that were joining leave again and join afresh later.
   */
  private sync(): void {
    const { entered, left } = scan(this.scene, this.members);
    cleanup(
      left.map((object) => () => this.leave(object)),
      "Physics object removal failed",
    );
    const joining: Member[] = [];
    try {
      for (const object of entered) {
        claim(object, this);
        // A listener of the previous world may have had this world take it already.
        if (this.members.has(object)) continue;
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
    for (const object of joining) setJoined(object, this);
  }
  /** Removes a member from the world and the backend, even if keeping its velocity fails. */
  private leave(object: Member): void {
    cleanup(
      [
        () => this.release(object),
        () => {
          if (!(object instanceof Joint)) this.interactions.remove(object);
          this.remove(object);
        },
      ],
      "Physics object removal failed",
    );
  }
  /** Ends membership; a dynamic body keeps its simulated velocity, unchecked, as its authored one. */
  private release(object: Member): void {
    this.members.delete(object);
    try {
      if (object instanceof RigidBody && object.bodyType === "dynamic")
        keepVelocity(object, this.readVelocity(object));
    } finally {
      setJoined(object, undefined);
    }
  }
  /** Joins `object` unless it is a member still under the scene, then throws unless it is one. */
  private join(object: Member): void {
    this.assertLive();
    if (object.world !== this || !holds(this.scene, object)) this.refresh();
    this.assertMember(object);
  }
  private assertMember(object: Member): void {
    // A listener may have disposed the world during the refresh before this.
    this.assertLive();
    if (object.world !== this)
      throw new Error(
        `Physics object ${object.name || object.type} is outside the world's scene`,
      );
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
}

/**
 * Decomposes the meshes under a new world's scene and brings the world up to date with it,
 * disposing the world if either fails.
 */
export async function build<World extends PhysicsWorld>(
  world: World,
): Promise<World> {
  try {
    await world.decompose(world.scene);
    // Claimed once decomposed, so commands during the build still act on authored state.
    claimScene(world);
    world.update(0);
  } catch (error) {
    rollback(error, [() => world.dispose()], "Physics world build failed");
  }
  return world;
}
