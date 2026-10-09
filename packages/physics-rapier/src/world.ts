import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  Trigger,
  PhysicsWorld,
  type Joint,
  type JointReading,
  type PhysicsWorldOptions,
  type PhysicsVelocity,
  type RaycastHit,
  type RaycastOptions,
  splitTransform,
} from "@drawcall/physics";
import { assembly } from "@drawcall/physics/backend";
import { Matrix4, Quaternion, Vector3 } from "three";
import { BodyBinding, prepareBody, writePose, writeBack } from "./body.js";
import { applyDrives } from "./drive.js";
import { sampleInteractions } from "./interactions.js";
import { RapierJointBinding, prepareJoint } from "./joint.js";
import { raycast, type Owner } from "./query.js";
import { TriggerBinding, prepareTrigger, releaseTrigger } from "./trigger.js";

export type RapierWorldOptions = PhysicsWorldOptions;

export class RapierWorld extends PhysicsWorld {
  private readonly native: Rapier.World;
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, RapierJointBinding>();
  private readonly triggers = new Map<Trigger, TriggerBinding>();
  /** The fixed body that stands in for `body0: null`. */
  private ground: Rapier.RigidBody | undefined;

  constructor(
    private readonly api: typeof Rapier,
    options: RapierWorldOptions,
  ) {
    super(options);
    this.native = new api.World(new Vector3(...this.gravity));
    if (this.solverIterations !== undefined)
      this.native.numSolverIterations = this.solverIterations;
    this.native.timestep = this.fixedDelta;
  }
  protected add(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody)
      this.bodies.set(object, new BodyBinding(this.api, this.native, object));
    else if (object instanceof Trigger)
      this.triggers.set(object, new TriggerBinding());
    else {
      const { body0, body1 } = object.options;
      const native0 = body0
        ? this.binding(body0).native
        : (this.ground ??= this.native.createRigidBody(
            this.api.RigidBodyDesc.fixed(),
          ));
      const bodies = [native0, this.binding(body1).native] as const;
      this.joints.set(object, new RapierJointBinding(object, bodies));
    }
  }
  protected remove(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody) {
      this.native.removeRigidBody(this.binding(object).native);
      this.bodies.delete(object);
    } else if (object instanceof Trigger) {
      releaseTrigger(this.native, need(this.triggers, object));
      this.triggers.delete(object);
    } else {
      const { native } = need(this.joints, object);
      if (native) this.native.removeImpulseJoint(native, true);
      this.joints.delete(object);
    }
  }
  protected prepare(): void {
    for (const [object, binding] of this.bodies)
      prepareBody(this.api, this.native, object, binding);
    for (const [object, binding] of this.triggers)
      prepareTrigger(this.api, this.native, object, binding, this.bodies);
    for (const binding of this.joints.values())
      prepareJoint(this.api, this.native, binding);
  }
  protected step(): void {
    for (const [object, { native }] of this.joints)
      if (native) applyDrives(object, native);
    this.native.step();
    for (const [object, { native: body }] of this.bodies) {
      writeBack(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const binding of this.joints.values()) binding.track();
    sampleInteractions(this.interactions, this.native, this.owners());
  }
  protected restore(): void {
    for (const [object, binding] of this.bodies) {
      object.validate();
      const body = binding.native;
      writePose(body, binding.initialPose);
      body.setLinvel(binding.initialVelocity.linear, true);
      body.setAngvel(binding.initialVelocity.angular, true);
      if (object.bodyType === "kinematic") {
        body.setNextKinematicTranslation(body.translation());
        body.setNextKinematicRotation(body.rotation());
      }
      body.resetForces(false);
      body.resetTorques(false);
      writeBack(object, body);
    }
    for (const binding of this.joints.values()) binding.rebase();
  }
  protected free(): void {
    this.native.free();
  }
  protected cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ): RaycastHit | null {
    this.native.propagateModifiedBodyPositionsToColliders();
    return raycast(
      this.api,
      this.native,
      this.owners(),
      origin,
      direction,
      maxDistance,
      options,
    );
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    const body = this.body(object);
    return {
      linear: new Vector3().copy(body.linvel()),
      angular: new Vector3().copy(body.angvel()),
    };
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    const body = this.body(object);
    if (value.linear) body.setLinvel(value.linear, true);
    if (value.angular) body.setAngvel(value.angular, true);
  }
  teleport(object: RigidBody): void {
    this.assertMember(object);
    const moved = assembly(object, this.joints.keys());
    for (const body of moved)
      writePose(
        this.binding(body).native,
        splitTransform(body.matrixWorld).pose,
      );
    // Joints inside the assembly keep their turns; only those it straddles start over.
    for (const [joint, binding] of this.joints) {
      const ends = [joint.options.body0, joint.options.body1].filter(
        (body) => body && moved.has(body),
      ).length;
      if (ends === 1) binding.rebase();
    }
  }
  setKinematicTarget(object: RigidBody, matrix: Matrix4): void {
    const body = this.body(object);
    body.setNextKinematicTranslation(
      new Vector3().setFromMatrixPosition(matrix),
    );
    body.setNextKinematicRotation(
      new Quaternion().setFromRotationMatrix(matrix),
    );
  }
  applyImpulse(object: RigidBody, impulse: Vector3, point?: Vector3): void {
    const body = this.body(object);
    if (point) body.applyImpulseAtPoint(impulse, point, true);
    else body.applyImpulse(impulse, true);
  }
  applyForce(object: RigidBody, force: Vector3, point?: Vector3): void {
    const body = this.body(object);
    if (point) body.addForceAtPoint(force, point, true);
    else body.addForce(force, true);
  }
  wake(object: RigidBody): void {
    this.body(object).wakeUp();
  }
  sleep(object: RigidBody): void {
    this.body(object).sleep();
  }
  readJoint(object: Joint): JointReading {
    this.assertMember(object);
    return need(this.joints, object).read({
      velocity: (body) => this.getVelocity(body),
      velocityAt: (body, point) =>
        new Vector3().copy(this.body(body).velocityAtPoint(point)),
    });
  }
  /** The Rapier body of a member. */
  private body(object: RigidBody): Rapier.RigidBody {
    this.assertMember(object);
    return this.binding(object).native;
  }
  private binding(object: RigidBody): BodyBinding {
    return need(this.bodies, object);
  }
  /** The scene owner of every Rapier collider. */
  private owners(): Map<number, Owner> {
    const owners = new Map<number, Owner>();
    for (const [body, { sources }] of this.bodies)
      for (const [handle, object] of sources)
        owners.set(handle, { kind: "body", body, object });
    for (const [trigger, { sources }] of this.triggers)
      for (const [handle, object] of sources)
        owners.set(handle, { kind: "trigger", trigger, object });
    return owners;
  }
}

/** Every member has a binding from the moment it is added. */
function need<K, V>(bindings: ReadonlyMap<K, V>, object: K): V {
  const binding = bindings.get(object);
  if (!binding) throw new Error("Physics object has no Rapier binding");
  return binding;
}
