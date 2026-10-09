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
import { initialVelocity } from "@drawcall/physics/backend";
import { Matrix4, Quaternion, Vector3 } from "three";
import { BodyBinding, prepareBody, writePose, writeBack } from "./body.js";
import { applyDrives } from "./drive.js";
import { sampleInteractions } from "./interactions.js";
import { RapierJointBinding, prepareJoint } from "./joint.js";
import { raycast, type Owner } from "./query.js";
import { TriggerBinding, prepareTrigger, releaseTrigger } from "./trigger.js";

/** Rapier collides triangle meshes on moving bodies as convex parts. */
const decomposes = (body: RigidBody) => body.bodyType !== "static";

export class RapierWorld extends PhysicsWorld {
  private readonly simulation: Rapier.World;
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, RapierJointBinding>();
  private readonly triggers = new Map<Trigger, TriggerBinding>();
  /** The fixed body that stands in for `body0: null`. */
  private readonly ground: Rapier.RigidBody;

  constructor(
    private readonly api: typeof Rapier,
    options: PhysicsWorldOptions,
  ) {
    super(options, decomposes);
    this.simulation = new api.World(new Vector3(...this.gravity));
    if (this.solverIterations !== undefined)
      this.simulation.numSolverIterations = this.solverIterations;
    this.simulation.timestep = this.fixedDelta;
    this.ground = this.simulation.createRigidBody(api.RigidBodyDesc.fixed());
  }
  protected add(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody)
      this.bodies.set(
        object,
        new BodyBinding(this.api, this.simulation, object),
      );
    else if (object instanceof Trigger)
      this.triggers.set(object, new TriggerBinding());
    else {
      const { body0, body1 } = object.options;
      const bodies = [
        body0 ? this.body(body0) : this.ground,
        this.body(body1),
      ] as const;
      this.joints.set(object, new RapierJointBinding(object, bodies));
    }
  }
  protected remove(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody) {
      this.simulation.removeRigidBody(this.body(object));
      this.bodies.delete(object);
    } else if (object instanceof Trigger) {
      releaseTrigger(this.simulation, need(this.triggers, object));
      this.triggers.delete(object);
    } else {
      const { native } = need(this.joints, object);
      if (native) this.simulation.removeImpulseJoint(native, true);
      this.joints.delete(object);
    }
  }
  protected prepare(): void {
    for (const [object, binding] of this.bodies)
      prepareBody(this.api, this.simulation, object, binding);
    for (const [object, binding] of this.triggers)
      prepareTrigger(this.api, this.simulation, object, binding, this.bodies);
    for (const binding of this.joints.values())
      prepareJoint(this.api, this.simulation, binding);
  }
  protected step(): void {
    for (const [object, { native }] of this.joints)
      if (native) applyDrives(object, native);
    this.simulation.step();
    for (const [object, { native: body }] of this.bodies) {
      writeBack(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const binding of this.joints.values()) binding.track();
    sampleInteractions(this.interactions, this.simulation, this.owners());
  }
  protected restore(): void {
    for (const [object, binding] of this.bodies) {
      object.validate();
      const body = binding.native;
      const velocity = initialVelocity(object);
      writePose(body, binding.pose);
      body.setLinvel(velocity.linear, true);
      body.setAngvel(velocity.angular, true);
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
    this.simulation.free();
  }
  protected cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options: RaycastOptions,
  ): RaycastHit | null {
    this.simulation.propagateModifiedBodyPositionsToColliders();
    return raycast(
      this.api,
      this.simulation,
      this.owners(),
      origin,
      direction,
      maxDistance,
      options,
    );
  }
  protected readVelocity(object: RigidBody): PhysicsVelocity {
    const body = this.body(object);
    return {
      linear: new Vector3().copy(body.linvel()),
      angular: new Vector3().copy(body.angvel()),
    };
  }
  protected writeVelocity(
    object: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void {
    const body = this.body(object);
    if (value.linear) body.setLinvel(value.linear, true);
    if (value.angular) body.setAngvel(value.angular, true);
  }
  protected writePoses(bodies: ReadonlySet<RigidBody>): void {
    for (const body of bodies)
      writePose(this.body(body), splitTransform(body.matrixWorld).pose);
    for (const binding of this.joints.values()) binding.rebaseIfSplit(bodies);
  }
  protected writeTarget(object: RigidBody, pose: Matrix4): void {
    const body = this.body(object);
    body.setNextKinematicTranslation(new Vector3().setFromMatrixPosition(pose));
    body.setNextKinematicRotation(new Quaternion().setFromRotationMatrix(pose));
  }
  protected writeImpulse(
    object: RigidBody,
    impulse: Vector3,
    point?: Vector3,
  ): void {
    const body = this.body(object);
    if (point) body.applyImpulseAtPoint(impulse, point, true);
    else body.applyImpulse(impulse, true);
  }
  protected writeForce(
    object: RigidBody,
    force: Vector3,
    point?: Vector3,
  ): void {
    const body = this.body(object);
    if (point) body.addForceAtPoint(force, point, true);
    else body.addForce(force, true);
  }
  protected writeSleeping(object: RigidBody, sleeping: boolean): void {
    if (sleeping) this.body(object).sleep();
    else this.body(object).wakeUp();
  }
  protected jointReading(object: Joint): JointReading {
    return need(this.joints, object).read({
      angular: (body) => new Vector3().copy(this.body(body).angvel()),
      velocityAt: (body, point) =>
        new Vector3().copy(this.body(body).velocityAtPoint(point)),
    });
  }
  /** The Rapier body of a member. */
  private body(object: RigidBody): Rapier.RigidBody {
    return need(this.bodies, object).native;
  }
  /** The scene owner of every Rapier collider. */
  private owners(): Map<number, Owner> {
    const owners = new Map<number, Owner>();
    for (const [body, { sources }] of this.bodies)
      for (const [collider, object] of sources)
        owners.set(collider.handle, { kind: "body", body, object });
    for (const [trigger, { sources }] of this.triggers)
      for (const [collider, object] of sources)
        owners.set(collider.handle, { kind: "trigger", trigger, object });
    return owners;
  }
}

/** Every member has a binding from the moment it joins. */
function need<K, V>(bindings: ReadonlyMap<K, V>, object: K): V {
  const binding = bindings.get(object);
  if (!binding) throw new Error("Physics object has no Rapier binding");
  return binding;
}
