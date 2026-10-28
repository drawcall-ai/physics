import type { Matrix4, Object3D, Vector3 } from "three";
import { type RigidBody, assertBodyType } from "./rigid/body.js";
import { validateVelocity, type PhysicsVelocity } from "./rigid/velocity.js";
import { Joint } from "./joints/joint.js";
import type { JointReading } from "./joints/reading.js";
import { placeAssembly } from "./joints/assembly.js";
import type { Trigger } from "./trigger.js";
import type { Vec3 } from "./colliders/collider.js";
import { settle, type Decomposes } from "./colliders/decomposition/parts.js";
import {
  rayDirection,
  type RaycastHit,
  type RaycastOptions,
} from "./raycast.js";
import { assertRigidTransform, validateVector } from "./transforms.js";
import { cleanup, rollback } from "./cleanup.js";
import { Interactions, type Contact } from "./interactions.js";
import type { Body } from "./body.js";
import { SoftBody } from "./soft/body.js";
import { SoftAttachment } from "./soft/attachment.js";
import {
  validateCapsule,
  type CapsuleShape,
  type CapsuleQueryOptions,
  type CapsuleContact,
  type CapsuleCastHit,
} from "./capsule.js";
import {
  claim,
  claimScene,
  commandWorld,
  releaseScene,
  scan,
  setJoined,
  type Member,
} from "./membership.js";

export interface PhysicsWorldOptions {
  /** The world simulates the bodies, joints and triggers under this object, normally the three.js Scene. */
  readonly scene: Object3D;
  /** m/s²; `[0, -9.81, 0]` when omitted. */
  readonly gravity?: Vec3;
  /** Fixed step in seconds; `1 / 60` when omitted. */
  readonly fixedDelta?: number;
  /** Most steps one `update` takes, 5 when omitted; elapsed time beyond them is dropped. */
  readonly maxSubsteps?: number;
  /** Constraint solver iterations per step; the backend's own default when omitted. */
  readonly solverIterations?: number;
}

/** The live reading of a joint, from the world that simulates it; for joints. */
export let readJoint: (joint: Joint) => JointReading;

/**
 * The fixed-step clock, step callbacks, event dispatch and command validation every backend
 * shares. Before every step and query, the world syncs its membership with the objects under its
 * scene and prepares the members, so every member has live backend state.
 * A scene has one live world; commands require membership in a built world. An invalid scene
 * member blocks synchronization until it is fixed or removed.
 * Contact and trigger events dispatch after each step. Listener failures are reported after
 * all queued events are delivered; removal emits ending events, while reset and disposal do not.
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
    readJoint = (joint) => {
      const world = commandWorld(joint);
      world.join(joint);
      return world.jointReading(joint);
    };
  }
  /** Simulated seconds of the completed steps. */
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
  protected capsuleContactQuery(
    _shape: CapsuleShape,
    _pose: Matrix4,
    _maxDistance: number,
    _options: CapsuleQueryOptions,
  ): CapsuleContact[] {
    throw new Error("This backend does not support capsule contacts");
  }
  protected capsuleCastQuery(
    _shape: CapsuleShape,
    _pose: Matrix4,
    _displacement: Vector3,
    _options: CapsuleQueryOptions,
  ): CapsuleCastHit | null {
    throw new Error("This backend does not support capsule casts");
  }
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

  protected readSoftPositions(_body: SoftBody): Float32Array {
    throw new Error("This backend does not support soft-body state");
  }
  protected writeSoftForce(
    _body: SoftBody,
    _force: Vector3,
    _point?: Vector3,
  ): void {
    throw new Error("This backend does not support soft-body forces");
  }
  protected writeSoftImpulse(
    _body: SoftBody,
    _impulse: Vector3,
    _point?: Vector3,
  ): void {
    throw new Error("This backend does not support soft-body impulses");
  }
  getSoftPositions(body: SoftBody): Float32Array {
    this.join(body);
    return this.readSoftPositions(body).slice();
  }

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
  applyImpulse(body: Body, impulse: Vector3, point?: Vector3): void {
    validateVector(impulse);
    if (point) validateVector(point);
    if (!(body instanceof SoftBody))
      assertBodyType(body, "dynamic", "An impulse");
    this.join(body);
    if (body instanceof SoftBody)
      return this.writeSoftImpulse(body, impulse, point);
    this.writeImpulse(body, impulse, point);
  }
  applyForce(body: Body, force: Vector3, point?: Vector3): void {
    validateVector(force);
    if (point) validateVector(point);
    if (!(body instanceof SoftBody)) assertBodyType(body, "dynamic", "A force");
    this.join(body);
    if (body instanceof SoftBody)
      return this.writeSoftForce(body, force, point);
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
  /** The body's contacts with other bodies in the last step, one per point; none before the first. */
  getContacts(body: Body): Contact[] {
    this.assertLive();
    this.refresh();
    this.assertMember(body);
    return this.interactions.contactsOf(body);
  }
  getOverlappingBodies(trigger: Trigger): Body[] {
    this.assertLive();
    this.refresh();
    this.assertMember(trigger);
    return this.interactions.bodies(trigger);
  }
  /** The closest hit, or null; a ray starting inside a shape hits its exit surface. */
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
  /** Contacts within `maxDistance` of a capsule at an arbitrary rigid pose. */
  capsuleContacts(
    shape: CapsuleShape,
    pose: Matrix4,
    maxDistance: number,
    options: CapsuleQueryOptions = {},
  ): CapsuleContact[] {
    validateCapsule(shape, pose, options);
    if (!Number.isFinite(maxDistance) || maxDistance < 0)
      throw new Error(
        "Capsule contact distance must be finite and nonnegative",
      );
    this.assertLive();
    this.refresh();
    this.assertLive();
    return this.capsuleContactQuery(shape, pose, maxDistance, options);
  }
  /**
   * Sweeps a capsule by a world-space translation, without rotating it. Returns the first
   * blocking contact; touching or penetrating convex surfaces allow motion away or tangent.
   * Zero translation has no hit; use capsuleContacts to inspect the starting pose.
   */
  castCapsule(
    shape: CapsuleShape,
    pose: Matrix4,
    displacement: Vector3,
    options: CapsuleQueryOptions = {},
  ): CapsuleCastHit | null {
    validateCapsule(shape, pose, options);
    validateVector(displacement);
    if (!Number.isFinite(displacement.length()))
      throw new Error("Capsule translation length must be finite");
    this.assertLive();
    this.refresh();
    this.assertLive();
    if (displacement.lengthSq() === 0) return null;
    return this.capsuleCastQuery(shape, pose, displacement, options);
  }
  /**
   * Decomposes the triangle meshes under `root` that this backend collides as convex parts, and
   * resolves once their parts arrive, so the next update collides them. Until then, meshes that
   * join or change after the build collide as their convex hull. Rejects if decomposing one
   * failed.
   */
  async decompose(root: Object3D = this.scene): Promise<void> {
    this.assertLive();
    await settle(root, this.decomposes);
  }
  /**
   * Advances by `delta` elapsed seconds in fixed steps. Throws when called from a step callback or
   * event listener. A step that throws ends the update: the steps before it count, the rest of
   * its time is dropped.
   */
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
  /**
   * Syncs, then returns the members to the pose and velocity they joined with, clearing contacts
   * and overlaps without events; `time` restarts at 0. Throws when called from a step callback or
   * event listener.
   * Scene membership, step callbacks, drive targets, and application state remain unchanged.
   */
  reset(): void {
    this.assertIdle();
    this.refresh();
    if (this.isDisposed) return;
    this.interactions.clear();
    this.restore();
    this.elapsed = 0;
    this.completed = 0;
  }
  /** Frees the world; its members leave it. */
  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    this.interactions.clear();
    this.before.clear();
    this.after.clear();
    releaseScene(this);
    for (const object of this.members) setJoined(object, undefined);
    this.members.clear();
    if (!this.busy) this.free();
  }
  /**
   * Calls `callback` with `fixedDelta` before each step, from the next step on; objects it adds
   * under the scene take part in that step. Returns an unsubscribe function.
   */
  onBeforeStep(callback: (delta: number) => void): () => void {
    this.assertLive();
    this.before.add(callback);
    return () => {
      this.before.delete(callback);
    };
  }
  /** Calls `callback` after each step and its events; returns an unsubscribe function. */
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
        claim(object, this, (owner) => owner.refresh());
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
  /** Removes a member from the world and the backend. */
  private leave(object: Member): void {
    this.members.delete(object);
    setJoined(object, undefined);
    if (!(object instanceof Joint) && !(object instanceof SoftAttachment))
      this.interactions.remove(object);
    this.remove(object);
  }
  /** Joins `object` unless it is a member, then throws unless it is one. */
  private join(object: Member): void {
    this.assertLive();
    if (object.world !== this) this.refresh();
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
    // Claimed once decomposed, so a new world never collides a mesh as its hull.
    await world.decompose();
    claimScene(world);
    world.update(0);
  } catch (error) {
    rollback(error, [() => world.dispose()], "Physics world build failed");
  }
  return world;
}
