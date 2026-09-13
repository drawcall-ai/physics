import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  clearDefaultWorld,
  RigidBody,
  AxisJoint,
  DistanceJoint,
  type Joint,
  type PhysicsWorld,
  type PhysicsOptions,
  type PhysicsVelocity,
  initialVelocity,
  setInitialVelocity,
  setWorldPose,
  authoredJointState,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import {
  synchronize,
  createBody,
  refreshBody,
  type BodyBinding,
} from "./body.js";
import { drive, joint, jointState } from "./joints.js";

export interface RapierOptions extends PhysicsOptions {
  solverIterations?: number;
}

type JointBinding = {
  body0: RigidBody | null;
  body1: RigidBody;
  target: Rapier.ImpulseJoint | undefined;
  frame0: Matrix4;
  frame1: Matrix4;
  anchors: string;
  configuration: string;
};

export class RapierWorld implements PhysicsWorld {
  private readonly backend: Rapier.World;
  private readonly objects = new Set<RigidBody>();
  private readonly constraints = new Set<Joint>();
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, JointBinding>();
  private anchor: Rapier.RigidBody | undefined;
  private elapsed = 0;
  private disposed = false;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  readonly fixedDelta: number;
  readonly maxSubsteps: number;

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
    else this.constraints.add(object);
  }
  unregister(object: RigidBody | Joint): void {
    if (object instanceof RigidBody) {
      for (const constraint of this.constraints) {
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
    this.constraints.delete(object);
    const target = this.joints.get(object)?.target;
    if (target) this.backend.removeImpulseJoint(target, true);
    this.joints.delete(object);
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
      this.elapsed -= this.fixedDelta;
    }
  }
  step(): void {
    this.assertActive();
    this.flush(true);
    for (const callback of this.before) callback(this.fixedDelta);
    this.flush();
    this.backend.step();
    for (const [object, { body }] of this.bodies) {
      synchronize(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const callback of this.after) callback(this.fixedDelta);
  }
  reset(): void {
    this.assertActive();
    for (const [object, { body, initial, velocity }] of this.bodies) {
      body.setTranslation(new Vector3().setFromMatrixPosition(initial), true);
      body.setRotation(new Quaternion().setFromRotationMatrix(initial), true);
      body.setLinvel(velocity.linear, true);
      body.setAngvel(velocity.angular, true);
      body.resetForces(false);
      body.resetTorques(false);
      synchronize(object, body);
    }
    this.elapsed = 0;
  }
  dispose(): void {
    if (this.disposed) return;
    for (const object of [...this.constraints, ...this.objects])
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
      : initialVelocity(object);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertObject(object);
    const body = this.bodies.get(object)?.body;
    if (!body) return setInitialVelocity(object, value);
    if (value.linear) body.setLinvel(value.linear, true);
    if (value.angular) body.setAngvel(value.angular, true);
  }
  teleport(object: RigidBody, matrix: Matrix4): void {
    this.assertObject(object);
    setWorldPose(object, matrix);
    const body = this.bodies.get(object)?.body;
    if (!body) return;
    body.setTranslation(new Vector3().setFromMatrixPosition(matrix), true);
    body.setRotation(new Quaternion().setFromRotationMatrix(matrix), true);
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
  getJointState(object: Joint) {
    this.assertObject(object);
    const binding = this.joints.get(object);
    if (binding?.target) return jointState(binding.target);
    return authoredJointState(
      object,
      binding ? [binding.frame0, binding.frame1] : undefined,
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
    for (const object of this.constraints) {
      if (pendingOnly && this.joints.has(object)) continue;
      if (
        !this.objects.has(object.options.body1) ||
        (object.options.body0 && !this.objects.has(object.options.body0))
      ) {
        throw new Error("Joint references a body outside this world.");
      }
      object.validate();
      const anchors = JSON.stringify([
        object.options.frame0?.elements,
        object.options.frame1?.elements,
      ]);
      let binding = this.joints.get(object);
      if (
        binding &&
        (binding.body0 !== object.options.body0 ||
          binding.body1 !== object.options.body1)
      ) {
        if (binding.target)
          this.backend.removeImpulseJoint(binding.target, true);
        this.joints.delete(object);
        binding = undefined;
      }
      if (binding && binding.anchors !== anchors)
        throw new Error(
          "Joint frames cannot change after creation; dispose and create a new joint.",
        );
      let target = binding?.target;
      const configuration = JSON.stringify(
        object instanceof DistanceJoint
          ? object.options.limits
          : object instanceof AxisJoint
            ? (object.options.axis ?? "Y")
            : null,
      );
      if (object.options.enabled === false) {
        if (target) this.backend.removeImpulseJoint(target, true);
        if (binding) binding.target = undefined;
        continue;
      }
      if (!target || binding?.configuration !== configuration) {
        const frame0 = binding?.frame0 ?? object.getFrame(0, new Matrix4());
        const frame1 = binding?.frame1 ?? object.getFrame(1, new Matrix4());
        if (!object.options.body0 && !this.anchor)
          this.anchor = this.backend.createRigidBody(
            this.api.RigidBodyDesc.fixed(),
          );
        const first = object.options.body0
          ? this.getBody(object.options.body0)
          : this.anchor;
        if (!first) throw new Error("Missing world anchor.");
        const replacement = joint(
          this.api,
          this.backend,
          object,
          frame0,
          frame1,
          first,
          this.getBody(object.options.body1),
        );
        if (target) this.backend.removeImpulseJoint(target, true);
        target = replacement;
        this.joints.set(object, {
          body0: object.options.body0,
          body1: object.options.body1,
          target,
          frame0,
          frame1,
          anchors,
          configuration,
        });
      }
      drive(this.api, object, target);
    }
  }
}
