import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  clearDefaultWorld,
  RigidBody,
  AxisJoint,
  type Joint,
  type PhysicsWorld,
  type PhysicsOptions,
  type PhysicsVelocity,
  type RaycastOptions,
  authoredVelocity,
  authoredJointState,
  setAuthoredVelocity,
  setWorldPose,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import {
  synchronize,
  createBody,
  refreshBody,
  type BodyBinding,
} from "./body.js";
import {
  applyEffort,
  prepareJoint,
  readJointState,
  sampleJoint,
  type JointBinding,
} from "./joints.js";
import { raycast } from "./query.js";

export interface RapierOptions extends PhysicsOptions {
  solverIterations?: number;
}

export class RapierWorld implements PhysicsWorld {
  private readonly backend: Rapier.World;
  private readonly objects = new Set<RigidBody>();
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, JointBinding | undefined>();
  private readonly pending = new Map<
    RigidBody,
    {
      velocity: PhysicsVelocity;
      changesVelocity: boolean;
      commands: ((body: Rapier.RigidBody) => void)[];
    }
  >();
  private readonly efforts = new Map<AxisJoint, number>();
  private anchor: Rapier.RigidBody | undefined;
  private elapsed = 0;
  private completedTime = 0;
  get time(): number {
    return this.completedTime;
  }
  private disposed = false;
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
  }
  register(object: RigidBody | Joint): void {
    this.assertActive();
    if (object.disposed) throw new Error("Cannot register a disposed object.");
    if (object.world !== this)
      throw new Error("Object belongs to another world.");
    if (object instanceof RigidBody) this.objects.add(object);
    else if (!this.joints.has(object)) this.joints.set(object, undefined);
  }
  unregister(object: RigidBody | Joint): void {
    if (object instanceof RigidBody) {
      for (const joint of this.joints.keys()) {
        if (joint.options.body0 === object || joint.options.body1 === object)
          joint.dispose();
      }
      this.objects.delete(object);
      this.pending.delete(object);
      const binding = this.bodies.get(object);
      if (binding) this.backend.removeRigidBody(binding.body);
      this.bodies.delete(object);
      return;
    }
    const target = this.joints.get(object)?.target;
    if (target) this.backend.removeImpulseJoint(target, true);
    this.joints.delete(object);
    if (object instanceof AxisJoint) this.efforts.delete(object);
  }
  update(delta: number): void {
    this.assertActive();
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative.");
    this.flush(true);
    this.elapsed = Math.min(
      this.elapsed + delta,
      this.fixedDelta * this.maxSubsteps,
    );
    while (this.elapsed >= this.fixedDelta) {
      this.step();
    }
  }
  private step(): void {
    this.assertActive();
    this.flush(true);
    for (const callback of this.before) callback(this.fixedDelta);
    this.flush();
    for (const object of this.efforts.keys()) {
      if (object.motor?.active)
        throw new Error("Disable the joint motor before applying effort");
    }
    for (const [object, effort] of this.efforts) {
      const target = this.joints.get(object)?.target;
      if (object.enabled && target) applyEffort(object, target, effort);
    }
    this.backend.step();
    this.efforts.clear();
    this.elapsed -= this.fixedDelta;
    this.completedTime += this.fixedDelta;
    for (const [object, { body }] of this.bodies) {
      synchronize(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const [object, binding] of this.joints) {
      if (binding) sampleJoint(object, binding);
    }
    for (const callback of this.after) callback(this.fixedDelta);
  }
  reset(): void {
    this.assertActive();
    this.pending.clear();
    this.efforts.clear();
    for (const [object, { body, initial, velocity }] of this.bodies) {
      object.validate();
      body.setTranslation(new Vector3().setFromMatrixPosition(initial), true);
      body.setRotation(new Quaternion().setFromRotationMatrix(initial), true);
      body.setLinvel(velocity.linear, true);
      body.setAngvel(velocity.angular, true);
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
      if (!binding) continue;
      sampleJoint(object, binding, true);
    }
  }
  dispose(): void {
    if (this.disposed) return;
    for (const object of [...this.joints.keys(), ...this.objects])
      object.dispose();
    this.backend.free();
    this.before.clear();
    this.after.clear();
    this.disposed = true;
    clearDefaultWorld(this);
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    this.assertObject(object);
    const body = this.bodies.get(object)?.body;
    if (body) return velocity(body);
    if (!this.pending.get(object)?.changesVelocity)
      return authoredVelocity(object);
    return this.preview([object], (bodies) => {
      const binding = bodies.get(object);
      if (!binding) throw new Error("Missing preview body");
      return velocity(binding.body);
    });
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertObject(object);
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
    this.assertObject(object);
    object.validate();
    setWorldPose(object, matrix);
    const body = this.bodies.get(object)?.body;
    if (!body) return;
    body.setTranslation(new Vector3().setFromMatrixPosition(matrix), true);
    body.setRotation(new Quaternion().setFromRotationMatrix(matrix), true);
    for (const [joint, binding] of this.joints) {
      if (
        binding &&
        (joint.options.body0 === object || joint.options.body1 === object)
      )
        sampleJoint(joint, binding, true);
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
  setJointEffort(object: AxisJoint, value: number): void {
    this.assertObject(object);
    if (object.enabled && value !== 0) this.efforts.set(object, value);
    else this.efforts.delete(object);
  }
  getJointState(object: Joint) {
    this.assertObject(object);
    const binding = this.joints.get(object);
    if (binding) return readJointState(object, binding);
    return authoredJointState(object, undefined, (body, point) => {
      const target = this.bodies.get(body)?.body;
      if (target) return new Vector3().copy(target.velocityAtPoint(point));
      if (body.getVelocity().angular.lengthSq() === 0)
        return body.getVelocity().linear;
      return this.preview([body], (bodies) => {
        const binding = bodies.get(body);
        if (!binding) throw new Error("Missing preview body");
        return new Vector3().copy(binding.body.velocityAtPoint(point));
      });
    });
  }
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ) {
    this.assertActive();
    for (const body of options?.excludeBodies ?? []) this.assertObject(body);
    for (const [object, binding] of this.bodies) {
      object.validate();
      refreshBody(this.api, this.backend, object, binding);
    }
    this.backend.propagateModifiedBodyPositionsToColliders();
    return this.preview(this.objects, (bodies, backend) =>
      raycast(
        this.api,
        backend,
        bodies,
        origin,
        direction,
        maxDistance,
        options,
      ),
    );
  }
  onBeforeStep(callback: (delta: number) => void): () => void {
    this.assertActive();
    this.before.add(callback);
    return () => {
      this.before.delete(callback);
    };
  }
  onAfterStep(callback: (delta: number) => void): () => void {
    this.assertActive();
    this.after.add(callback);
    return () => {
      this.after.delete(callback);
    };
  }
  private assertActive(): void {
    if (this.disposed) throw new Error("Physics world has been disposed.");
  }
  private assertObject(object: RigidBody | Joint): void {
    this.assertActive();
    if (object.disposed) throw new Error("Physics object has been disposed.");
    if (object.world !== this)
      throw new Error("Object belongs to another world.");
  }
  private command(
    object: RigidBody,
    command: (body: Rapier.RigidBody) => void,
    changesVelocity = false,
  ): void {
    this.assertObject(object);
    const body = this.bodies.get(object)?.body;
    if (body) return command(body);
    let pending = this.pending.get(object);
    if (!pending) {
      pending = {
        velocity: authoredVelocity(object),
        changesVelocity: false,
        commands: [],
      };
      this.pending.set(object, pending);
    }
    pending.changesVelocity ||= changesVelocity;
    pending.commands.push(command);
  }
  private replay(object: RigidBody, body: Rapier.RigidBody): void {
    const pending = this.pending.get(object);
    if (!pending) return;
    body.setLinvel(pending.velocity.linear, true);
    body.setAngvel(pending.velocity.angular, true);
    for (const command of pending.commands) command(body);
  }
  private preview<T>(
    objects: Iterable<RigidBody>,
    read: (bodies: Map<RigidBody, BodyBinding>, backend: Rapier.World) => T,
  ): T {
    const pending = [...objects].filter((object) => !this.bodies.has(object));
    if (!pending.length) return read(this.bodies, this.backend);
    const backend = new this.api.World(new Vector3());
    const bodies = new Map(this.bodies);
    try {
      for (const object of pending) {
        object.validate();
        const binding = createBody(this.api, backend, object);
        this.replay(object, binding.body);
        bodies.set(object, binding);
      }
      return read(bodies, backend);
    } finally {
      backend.free();
    }
  }
  private getBody(object: RigidBody): Rapier.RigidBody {
    this.assertObject(object);
    const binding = this.bodies.get(object);
    if (!binding) throw new Error("Missing prepared body");
    return binding.body;
  }
  private flush(pendingOnly = false): void {
    this.assertActive();
    for (const object of this.objects) {
      const binding = this.bodies.get(object);
      if (pendingOnly && binding) continue;
      object.validate();
      if (binding) refreshBody(this.api, this.backend, object, binding);
      else {
        const created = createBody(this.api, this.backend, object);
        this.bodies.set(object, created);
        this.replay(object, created.body);
        this.pending.delete(object);
      }
    }
    for (const [object, binding] of this.joints) {
      if (pendingOnly && binding) continue;
      if (!object.options.body0 && !this.anchor)
        this.anchor = this.backend.createRigidBody(
          this.api.RigidBodyDesc.fixed(),
        );
      const first = object.options.body0
        ? this.getBody(object.options.body0)
        : this.anchor;
      if (!first) throw new Error("Missing world anchor");
      this.joints.set(
        object,
        prepareJoint(
          this.api,
          this.backend,
          object,
          first,
          this.getBody(object.options.body1),
          binding,
        ),
      );
    }
  }
}

function velocity(body: Rapier.RigidBody): PhysicsVelocity {
  return {
    linear: new Vector3().copy(body.linvel()),
    angular: new Vector3().copy(body.angvel()),
  };
}
