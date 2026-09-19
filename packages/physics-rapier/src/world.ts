import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  Trigger,
  SteppedWorld,
  type Joint,
  type PhysicsOptions,
  type PhysicsVelocity,
  type RaycastOptions,
  authoredVelocity,
  sceneJointReading,
  assembly,
  setAuthoredVelocity,
  splitTransform,
  assertLive,
  assertOwned,
  ancestorBody,
  cleanup,
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
import { sampleInteractions } from "./interactions.js";
import {
  refreshTrigger,
  removeTrigger,
  type TriggerBinding,
} from "./triggers.js";

export type RapierOptions = PhysicsOptions;

export class RapierWorld extends SteppedWorld {
  private readonly backend: Rapier.World;
  private readonly objects = new Set<RigidBody>();
  private readonly bodies = new Map<RigidBody, BodyBinding>();
  private readonly joints = new Map<Joint, JointBinding | undefined>();
  private readonly triggers = new Map<Trigger, TriggerBinding | undefined>();
  private freed = false;
  private readonly pending: Pending;
  private anchor: Rapier.RigidBody | undefined;

  constructor(
    private readonly api: typeof Rapier,
    options: RapierOptions = {},
  ) {
    super(options);
    this.backend = new api.World(new Vector3(...this.gravity));
    if (this.solverIterations !== undefined)
      this.backend.numSolverIterations = this.solverIterations;
    this.backend.timestep = this.fixedDelta;
    this.pending = new Pending(api, this.bodies);
  }
  register(object: RigidBody | Joint | Trigger): void {
    assertOwned(this, object);
    if (object instanceof RigidBody) this.objects.add(object);
    else if (object instanceof Trigger) {
      if (!this.triggers.has(object)) this.triggers.set(object, undefined);
    } else if (!this.joints.has(object)) this.joints.set(object, undefined);
  }
  unregister(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody)
      for (const trigger of this.triggers.keys())
        if (ancestorBody(trigger) === object) this.interactions.remove(trigger);
    if (object instanceof RigidBody || object instanceof Trigger)
      this.interactions.remove(object);
    cleanup(
      [() => this.remove(object), () => this.dispatch()],
      "Physics object removal failed",
    );
  }
  private remove(object: RigidBody | Joint | Trigger): void {
    if (object instanceof Trigger) {
      const binding = this.triggers.get(object);
      if (binding) removeTrigger(this.backend, binding);
      this.triggers.delete(object);
      return;
    }
    if (object instanceof RigidBody) {
      cleanup(
        [
          ...[...this.triggers]
            .filter(([trigger]) => ancestorBody(trigger) === object)
            .map(([trigger, binding]) => () => {
              if (binding) removeTrigger(this.backend, binding);
              this.triggers.set(trigger, undefined);
            }),
          () => {
            this.objects.delete(object);
            this.pending.delete(object);
            const binding = this.bodies.get(object);
            if (binding) this.backend.removeRigidBody(binding.body);
            this.bodies.delete(object);
          },
        ],
        "Rigid body removal failed",
      );
      return;
    }
    const joint = this.joints.get(object)?.joint;
    if (joint) this.backend.removeImpulseJoint(joint, true);
    this.joints.delete(object);
  }
  protected prepare(): void {
    this.reconcile("pending");
  }
  protected step(): void {
    this.reconcile("all");
    for (const [object, binding] of this.joints) {
      if (binding?.joint && object.enabled) applyEfforts(object, binding.joint);
    }
    this.backend.step();
    for (const [object, { body }] of this.bodies) {
      synchronize(object, body);
      body.resetForces(false);
      body.resetTorques(false);
    }
    for (const [object, binding] of this.joints) {
      if (binding) trackAngle(object, binding);
    }
    const triggers = new Map<Trigger, TriggerBinding>();
    for (const [trigger, binding] of this.triggers) {
      if (!binding) throw new Error("Missing prepared trigger");
      triggers.set(trigger, binding);
    }
    sampleInteractions(this.interactions, this.backend, this.bodies, triggers);
  }
  protected restore(): void {
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
    for (const [object, binding] of this.joints) {
      if (binding) rebaseAngle(object, binding);
    }
  }
  protected disposeObjects(): void {
    cleanup(
      [...this.triggers.keys(), ...this.joints.keys(), ...this.objects].map(
        (object) => () => object.dispose(),
      ),
      "Physics world disposal failed",
    );
  }
  protected free(): void {
    if (this.freed) return;
    this.backend.free();
    this.freed = true;
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
  teleport(object: RigidBody): void {
    assertOwned(this, object);
    const moved = assembly(object, this.joints.keys());
    for (const body of moved) {
      const backend = this.bodies.get(body)?.body;
      if (backend) writePose(backend, splitTransform(body.matrixWorld).pose);
    }
    // Joints inside the assembly keep their turns; only those it straddles start over.
    for (const [joint, binding] of this.joints) {
      const ends = [joint.options.body0, joint.options.body1].filter(
        (body) => body && moved.has(body),
      ).length;
      if (binding && ends === 1) rebaseAngle(joint, binding);
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
    return sceneJointReading(object, undefined, (body, point) => {
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
      if (object.disposed) continue;
      object.validate();
      refreshBody(this.api, this.backend, object, binding);
    }
    this.backend.propagateModifiedBodyPositionsToColliders();
    return this.pending.preview(this.objects, (bodies) =>
      raycast(
        this.api,
        bodies,
        origin,
        direction,
        maxDistance,
        options,
        [...this.triggers.keys()].filter((trigger) => !trigger.disposed),
      ),
    );
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
  private reconcile(scope: "pending" | "all"): void {
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
    for (const [object, binding] of this.triggers) {
      if (binding && scope === "pending") continue;
      this.triggers.set(
        object,
        refreshTrigger(this.api, this.backend, object, this.bodies, binding),
      );
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
