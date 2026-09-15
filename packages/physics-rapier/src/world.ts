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
import { Constraints } from "./constraints.js";
import { raycast } from "./query.js";

export interface RapierOptions extends PhysicsOptions {
  solverIterations?: number;
}

export class RapierWorld implements PhysicsWorld {
  private readonly backend: Rapier.World;
  private readonly objects = new Set<RigidBody>();
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints: Constraints;
  private elapsed = 0;
  private completedTime = 0;
  get time(): number {
    return this.completedTime;
  }
  private disposed = false;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  private readonly timestep: number;
  get fixedDelta(): number {
    return this.timestep;
  }
  private readonly maxSubsteps: number;

  constructor(
    private readonly api: typeof Rapier,
    options: RapierOptions = {},
  ) {
    this.timestep = options.fixedDelta ?? 1 / 60;
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
    this.joints = new Constraints(api, this.backend, (object) =>
      this.getBody(object),
    );
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
    else this.joints.objects.add(object);
  }
  unregister(object: RigidBody | Joint): void {
    if (object instanceof RigidBody) {
      for (const constraint of this.joints.objects) {
        if (
          constraint.options.body0 === object ||
          constraint.options.body1 === object
        )
          constraint.dispose();
      }
      this.objects.delete(object);
      const binding = this.bodies.get(object);
      if (binding) this.backend.removeRigidBody(binding.body);
      this.bodies.delete(object);
      return;
    }
    this.joints.remove(object);
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
    this.joints.applyEfforts();
    this.backend.step();
    this.joints.clearEfforts();
    this.elapsed -= this.fixedDelta;
    this.completedTime += this.fixedDelta;
    for (const [object, { body }] of this.bodies) {
      synchronize(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    this.joints.sampleAngles();
    for (const callback of this.after) callback(this.fixedDelta);
  }
  reset(): void {
    this.assertActive();
    for (const [object, { body, initial, velocity }] of this.bodies) {
      object.validate();
      body.setTranslation(new Vector3().setFromMatrixPosition(initial), true);
      body.setRotation(new Quaternion().setFromRotationMatrix(initial), true);
      body.setLinvel(velocity.linear, true);
      body.setAngvel(velocity.angular, true);
      body.resetForces(false);
      body.resetTorques(false);
      synchronize(object, body);
    }
    this.elapsed = 0;
    this.completedTime = 0;
    this.joints.clearEfforts();
    this.joints.sampleAngles(true);
  }
  dispose(): void {
    if (this.disposed) return;
    for (const object of [...this.joints.objects, ...this.objects])
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
    return body
      ? {
          linear: new Vector3().copy(body.linvel()),
          angular: new Vector3().copy(body.angvel()),
        }
      : authoredVelocity(object);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertObject(object);
    const body = this.bodies.get(object)?.body;
    if (!body) return setAuthoredVelocity(object, value);
    if (value.linear) body.setLinvel(value.linear, true);
    if (value.angular) body.setAngvel(value.angular, true);
  }
  teleport(object: RigidBody, matrix: Matrix4): void {
    this.assertObject(object);
    object.validate();
    setWorldPose(object, matrix);
    const body = this.bodies.get(object)?.body;
    if (!body) return;
    body.setTranslation(new Vector3().setFromMatrixPosition(matrix), true);
    body.setRotation(new Quaternion().setFromRotationMatrix(matrix), true);
    this.joints.sampleAngles(true, object);
  }
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void {
    const body = this.getBody(object);
    body.setNextKinematicTranslation(
      new Vector3().setFromMatrixPosition(matrix),
    );
    body.setNextKinematicRotation(
      new Quaternion().setFromRotationMatrix(matrix),
    );
  }
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void {
    const body = this.getBody(object);
    if (point) body.applyImpulseAtPoint(impulse, point, true);
    else body.applyImpulse(impulse, true);
  }
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void {
    const body = this.getBody(object);
    if (point) body.addForceAtPoint(force, point, true);
    else body.addForce(force, true);
  }
  wake(object: RigidBody): void {
    this.getBody(object).wakeUp();
  }
  sleep(object: RigidBody): void {
    this.getBody(object).sleep();
  }
  setJointEffort(object: AxisJoint, value: number): void {
    this.assertObject(object);
    this.joints.setEffort(object, value);
  }
  getJointState(object: Joint) {
    this.assertObject(object);
    return this.joints.getState(object);
  }
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ) {
    this.assertActive();
    for (const body of options?.excludeBodies ?? []) this.assertObject(body);
    return raycast(
      this.api,
      this.backend,
      this.bodies,
      origin,
      direction,
      maxDistance,
      options,
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
  private getBody(object: RigidBody): Rapier.RigidBody {
    this.assertObject(object);
    const binding = this.bodies.get(object);
    if (!binding)
      throw new Error(
        "Body backend is not initialized; finish assembly and call world.update(0) before simulation operations.",
      );
    return binding.body;
  }
  private flush(pendingOnly = false): void {
    this.assertActive();
    for (const object of this.objects) {
      const binding = this.bodies.get(object);
      if (pendingOnly && binding) continue;
      object.validate();
      if (binding) refreshBody(this.api, this.backend, object, binding);
      else this.bodies.set(object, createBody(this.api, this.backend, object));
    }
    this.joints.refresh(pendingOnly);
  }
}
