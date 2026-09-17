import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  type Joint,
  type PhysicsWorld,
  type PhysicsOptions,
  type PhysicsVelocity,
  type RaycastOptions,
  clearDefaultWorld,
  authoredVelocity,
  authoredJointReading,
  setAuthoredVelocity,
  setWorldPose,
  assertLive,
  assertOwned,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import {
  synchronize,
  createBody,
  refreshBody,
  writePose,
  type BodyBinding,
} from "./body.js";
import { applyEfforts } from "./drives.js";
import { prepareJoint, type JointBinding } from "./joints.js";
import { Pending, type Command } from "./pending.js";
import { raycast } from "./query.js";
import { readBinding, rebaseAngle, trackAngle } from "./reading.js";

export interface RapierOptions extends PhysicsOptions {
  solverIterations?: number;
}

export class RapierWorld implements PhysicsWorld {
  private readonly backend: Rapier.World;
  private readonly objects = new Set<RigidBody>();
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, JointBinding | undefined>();
  private readonly pending: Pending;
  private anchor: Rapier.RigidBody | undefined;
  private elapsed = 0;
  private completedTime = 0;
  get time(): number {
    return this.completedTime;
  }
  private isDisposed = false;
  get disposed(): boolean {
    return this.isDisposed;
  }
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  readonly fixedDelta: number;
  private readonly maxSubsteps: number;

  constructor(
    private readonly api: typeof Rapier,
    options: RapierOptions = {},
  ) {
    this.fixedDelta = options.fixedDelta ?? 1 / 60;
    this.maxSubsteps = options.maxSubsteps ?? 5;
    if (!Number.isFinite(this.fixedDelta) || this.fixedDelta <= 0)
      throw new Error("fixedDelta must be positive and finite.");
    if (!Number.isInteger(this.maxSubsteps) || this.maxSubsteps < 1)
      throw new Error("maxSubsteps must be a positive integer.");
    const gravity = new Vector3(...(options.gravity ?? [0, -9.81, 0]));
    if (![gravity.x, gravity.y, gravity.z].every(Number.isFinite))
      throw new Error("Gravity must be finite.");
    if (
      options.solverIterations !== undefined &&
      (!Number.isInteger(options.solverIterations) ||
        options.solverIterations < 1)
    )
      throw new Error("solverIterations must be a positive integer.");
    this.backend = new api.World(gravity);
    if (options.solverIterations !== undefined)
      this.backend.numSolverIterations = options.solverIterations;
    this.backend.timestep = this.fixedDelta;
    this.pending = new Pending(api, this.bodies);
  }
  register(object: RigidBody | Joint): void {
    assertOwned(this, object);
    if (object instanceof RigidBody) this.objects.add(object);
    else if (!this.joints.has(object)) this.joints.set(object, undefined);
  }
  unregister(object: RigidBody | Joint): void {
    if (object instanceof RigidBody) {
      for (const joint of this.joints.keys())
        if (joint.connects(object)) joint.dispose();
      this.objects.delete(object);
      this.pending.delete(object);
      const binding = this.bodies.get(object);
      if (binding) this.backend.removeRigidBody(binding.body);
      this.bodies.delete(object);
      return;
    }
    const joint = this.joints.get(object)?.joint;
    if (joint) this.backend.removeImpulseJoint(joint, true);
    this.joints.delete(object);
  }
  update(delta: number): void {
    assertLive(this);
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative.");
    this.prepare("pending");
    this.elapsed = Math.min(
      this.elapsed + delta,
      this.fixedDelta * this.maxSubsteps,
    );
    while (this.elapsed >= this.fixedDelta) {
      this.step();
    }
  }
  private step(): void {
    assertLive(this);
    this.prepare("pending");
    for (const callback of this.before) callback(this.fixedDelta);
    this.prepare("all");
    for (const [object, binding] of this.joints) {
      if (binding?.joint && object.enabled) applyEfforts(object, binding.joint);
    }
    this.backend.step();
    this.elapsed -= this.fixedDelta;
    this.completedTime += this.fixedDelta;
    for (const [object, { body }] of this.bodies) {
      synchronize(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const [object, binding] of this.joints) {
      if (binding) trackAngle(object, binding);
    }
    for (const callback of this.after) callback(this.fixedDelta);
  }
  reset(): void {
    assertLive(this);
    this.pending.clear();
    for (const [object, binding] of this.bodies) {
      object.validate();
      const { body } = binding;
      writePose(body, binding.initialPose);
      body.setLinvel(binding.initialVelocity.linear, true);
      body.setAngvel(binding.initialVelocity.angular, true);
      if (object.bodyType === "kinematic") {
        body.setNextKinematicTranslation(body.translation());
        body.setNextKinematicRotation(body.rotation());
      }
      body.resetForces(false);
      body.resetTorques(false);
      synchronize(object, body);
    }
    this.elapsed = 0;
    this.completedTime = 0;
    for (const [object, binding] of this.joints) {
      if (binding) rebaseAngle(object, binding);
    }
  }
  dispose(): void {
    if (this.isDisposed) return;
    for (const object of [...this.joints.keys(), ...this.objects])
      object.dispose();
    this.backend.free();
    this.before.clear();
    this.after.clear();
    this.isDisposed = true;
    clearDefaultWorld(this);
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    assertOwned(this, object);
    const body = this.bodies.get(object)?.body;
    if (body) return velocity(body);
    if (!this.pending.changesVelocity(object)) return authoredVelocity(object);
    return this.pending.previewBody(object, velocity);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    assertOwned(this, object);
    const linear = value.linear?.clone(),
      angular = value.angular?.clone();
    if (!this.bodies.has(object)) {
      setAuthoredVelocity(object, value);
      if (!this.pending.has(object)) return;
    }
    this.command(object, (body) => {
      if (linear) body.setLinvel(linear, true);
      if (angular) body.setAngvel(angular, true);
    });
  }
  teleport(object: RigidBody, matrix: Matrix4): void {
    assertOwned(this, object);
    object.validate();
    setWorldPose(object, matrix);
    const body = this.bodies.get(object)?.body;
    if (!body) return;
    writePose(body, matrix);
    for (const [joint, binding] of this.joints) {
      if (binding && joint.connects(object)) rebaseAngle(joint, binding);
    }
  }
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void {
    const position = new Vector3().setFromMatrixPosition(matrix);
    const rotation = new Quaternion().setFromRotationMatrix(matrix);
    this.command(object, (body) => {
      body.setNextKinematicTranslation(position);
      body.setNextKinematicRotation(rotation);
    });
  }
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void {
    const value = impulse.clone(),
      at = point?.clone();
    this.command(
      object,
      (body) => {
        if (at) body.applyImpulseAtPoint(value, at, true);
        else body.applyImpulse(value, true);
      },
      true,
    );
  }
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void {
    const value = force.clone(),
      at = point?.clone();
    this.command(object, (body) => {
      if (at) body.addForceAtPoint(value, at, true);
      else body.addForce(value, true);
    });
  }
  wake(object: RigidBody): void {
    this.command(object, (body) => body.wakeUp());
  }
  sleep(object: RigidBody): void {
    this.command(object, (body) => body.sleep(), true);
  }
  readJoint(object: Joint) {
    assertOwned(this, object);
    const binding = this.joints.get(object);
    if (binding) return readBinding(object, binding);
    return authoredJointReading(object, undefined, (body, point) => {
      const target = this.bodies.get(body)?.body;
      if (target) return new Vector3().copy(target.velocityAtPoint(point));
      const { linear, angular } = body.getVelocity();
      if (angular.lengthSq() === 0) return linear;
      return this.pending.previewBody(body, (preview) =>
        new Vector3().copy(preview.velocityAtPoint(point)),
      );
    });
  }
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ) {
    assertLive(this);
    for (const body of options?.excludeBodies ?? []) assertOwned(this, body);
    for (const [object, binding] of this.bodies) {
      object.validate();
      refreshBody(this.api, this.backend, object, binding);
    }
    this.backend.propagateModifiedBodyPositionsToColliders();
    return this.pending.preview(this.objects, (bodies) =>
      raycast(this.api, bodies, origin, direction, maxDistance, options),
    );
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
  private command(
    object: RigidBody,
    command: Command,
    changesVelocity = false,
  ): void {
    assertOwned(this, object);
    const body = this.bodies.get(object)?.body;
    if (body) command(body);
    else this.pending.push(object, command, changesVelocity);
  }
  /** Creates backend objects for unprepared bodies and joints; `"all"` also refreshes prepared ones. */
  private prepare(scope: "pending" | "all"): void {
    assertLive(this);
    for (const object of this.objects) {
      const binding = this.bodies.get(object);
      if (binding && scope === "pending") continue;
      object.validate();
      if (binding) {
        refreshBody(this.api, this.backend, object, binding);
        continue;
      }
      const created = createBody(this.api, this.backend, object);
      this.bodies.set(object, created);
      this.pending.replay(object, created.body);
      this.pending.delete(object);
    }
    for (const [object, binding] of this.joints) {
      if (binding && scope === "pending") continue;
      this.joints.set(
        object,
        prepareJoint(
          this.api,
          this.backend,
          object,
          this.jointBody(object.options.body0),
          this.jointBody(object.options.body1),
          binding,
        ),
      );
    }
  }
  /** The prepared body, or the shared fixed anchor that stands in for the world. */
  private jointBody(object: RigidBody | null): Rapier.RigidBody {
    if (!object) {
      this.anchor ??= this.backend.createRigidBody(
        this.api.RigidBodyDesc.fixed(),
      );
      return this.anchor;
    }
    assertOwned(this, object);
    const binding = this.bodies.get(object);
    if (!binding) throw new Error("Missing prepared body");
    return binding.body;
  }
}

function velocity(body: Rapier.RigidBody): PhysicsVelocity {
  return {
    linear: new Vector3().copy(body.linvel()),
    angular: new Vector3().copy(body.angvel()),
  };
}
