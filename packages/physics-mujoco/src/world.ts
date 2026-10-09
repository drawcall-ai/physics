import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  PhysicsWorld,
  splitTransform,
  type Trigger,
  type JointReading,
  type PhysicsVelocity,
  type RaycastOptions,
} from "@drawcall/physics";
import {
  assembly,
  rollback,
  setWorldPose,
  treeJoint,
  JointBinding,
} from "@drawcall/physics/backend";
import { Matrix4, type Object3D, type Vector3 } from "three";
import {
  compile,
  type ModelOptions,
  type Simulation,
} from "./model/compile.js";
import { carryState, type Initial } from "./model/state.js";
import { modelOptions, type MujocoWorldOptions } from "./options.js";
import { array, at, pose } from "./heap.js";
import {
  bodyId,
  motionOf,
  velocity,
  writeVelocity,
  writePose,
  writeBack,
  refreshPoses,
} from "./body.js";
import { damp, wrench } from "./force.js";
import { applyDrives } from "./drive.js";
import { raycast } from "./query.js";
import { sampleInteractions } from "./interactions.js";
import {
  fingerprint,
  lockScales,
  sameFingerprints,
  type Fingerprints,
} from "./changes.js";

/**
 * Simulates its members in one compiled MuJoCo model, which it rebuilds whenever membership or
 * an authored property the model bakes in changes.
 */
export class MujocoWorld extends PhysicsWorld {
  private readonly options: ModelOptions;
  /** Each body's pose and velocity when a model first held it, which reset returns it to. */
  private readonly bodies = new Map<RigidBody, Initial>();
  private readonly joints = new Map<Joint, JointBinding>();
  private readonly triggers = new Set<Trigger>();
  /** Bodies and joints no model has held yet; a successful rebuild binds them. */
  private readonly added = new Set<RigidBody | Joint>();
  private native: Simulation;
  private fingerprints: Fingerprints = new Map();
  private scales = new Map<Object3D, Vector3>();
  // Forces and targets wait for the step: a rebuild while refreshing would discard their writes.
  private forces: { body: RigidBody; force: Vector3; point?: Vector3 }[] = [];
  private readonly targets = new Map<RigidBody, Matrix4>();
  constructor(
    private readonly api: MainModule,
    options: MujocoWorldOptions,
  ) {
    super(options);
    this.options = modelOptions(options, {
      fixedDelta: this.fixedDelta,
      gravity: this.gravity,
      solverIterations: this.solverIterations ?? 50,
    });
    this.native = compile(api, [], this.joints, [], this.options);
  }
  protected add(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody || object instanceof Joint)
      this.added.add(object);
    else this.triggers.add(object);
  }
  protected remove(object: RigidBody | Joint | Trigger): void {
    if (object instanceof Joint) {
      this.added.delete(object);
      this.joints.delete(object);
      return;
    }
    if (!(object instanceof RigidBody)) {
      this.triggers.delete(object);
      return;
    }
    this.added.delete(object);
    this.bodies.delete(object);
    this.targets.delete(object);
    this.forces = this.forces.filter(({ body }) => body !== object);
  }
  protected prepare(): void {
    const fingerprints = this.fingerprint();
    if (sameFingerprints(fingerprints, this.fingerprints)) return;
    this.rebuild(this.native, fingerprints);
  }
  protected step(): void {
    const { api, native } = this;
    for (const { body, force, point } of this.forces.splice(0))
      wrench(api, native, bodyId(native, body), force, { point });
    let moved = false;
    for (const [body, matrix] of this.targets) {
      const id = native.targets.get(body);
      if (id === undefined) throw new Error("Missing MuJoCo kinematic target");
      if (writePose(api, native, id, matrix)) moved = true;
    }
    this.targets.clear();
    refreshPoses(api, native, moved);
    damp(api, native, this.fixedDelta);
    applyDrives(api, native, this.joints, this.fixedDelta);
    // Split step: the state already holds step1 results for this pose (the last mj_step1 or any
    // mj_forward), so collision and the solve run once per step rather than in mj_step and mj_forward.
    api.mj_step2(native.model, native.data);
    api.mj_step1(native.model, native.data);
    checkState(api, native);
    array(native.data.qfrc_applied).fill(0);
    writeBack(native);
    for (const binding of this.joints.values()) binding.track();
    refreshPoses(api, native);
    sampleInteractions(api, native, this.interactions);
  }
  protected restore(): void {
    this.forces.length = 0;
    this.targets.clear();
    for (const [body, { pose }] of this.bodies) setWorldPose(body, pose);
    for (const binding of this.joints.values()) binding.rebase();
    // A rebuild without the previous model starts every body from its authored motion.
    this.rebuild(undefined, this.fingerprint());
  }
  protected free(): void {
    this.native.free();
  }
  getVelocity(body: RigidBody): PhysicsVelocity {
    this.assertMember(body);
    return velocity(this.api, this.native, bodyId(this.native, body));
  }
  setVelocity(body: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.assertMember(body);
    writeVelocity(this.api, this.native, bodyId(this.native, body), value);
    this.api.mj_forward(this.native.model, this.native.data);
  }
  teleport(body: RigidBody): void {
    this.assertMember(body);
    const { api, native } = this;
    const id = bodyId(native, body);
    this.targets.delete(body);
    const moved = assembly(body, this.joints.keys());
    const matrix = splitTransform(body.matrixWorld).pose;
    if (body.bodyType === "dynamic") {
      const anchored = [...this.joints.keys()].some(
        (joint) =>
          treeJoint(joint) &&
          moved.has(joint.options.body1) &&
          joint.options.body0?.bodyType !== "dynamic",
      );
      if (anchored)
        throw new Error(
          "MuJoCo cannot teleport a body articulated to the world or a static or kinematic base",
        );
      // The assembly moves rigidly, so the change of pose goes onto its root's free joint.
      const root = at(native.model.body_rootid, id);
      const delta = matrix
        .clone()
        .multiply(pose(native.data.xpos, native.data.xquat, id).invert());
      writePose(
        api,
        native,
        root,
        delta.multiply(pose(native.data.xpos, native.data.xquat, root)),
      );
    } else writePose(api, native, id, matrix);
    const target = native.targets.get(body);
    if (target !== undefined) writePose(api, native, target, matrix);
    api.mj_forward(native.model, native.data);
    writeBack(native);
    // Joints inside the assembly keep their turns; only those it straddles start over.
    for (const [joint, binding] of this.joints) {
      const { body0, body1 } = joint.options;
      if ((body0 !== null && moved.has(body0)) !== moved.has(body1))
        binding.rebase();
    }
  }
  setKinematicTarget(body: RigidBody, matrix: Matrix4): void {
    this.assertMember(body);
    this.targets.set(body, matrix.clone());
  }
  applyForce(body: RigidBody, force: Vector3, point?: Vector3): void {
    this.assertMember(body);
    this.forces.push({ body, force: force.clone(), point: point?.clone() });
  }
  applyImpulse(body: RigidBody, impulse: Vector3, point?: Vector3): void {
    this.assertMember(body);
    const { api, native } = this;
    wrench(api, native, bodyId(native, body), impulse, {
      point,
      impulse: true,
    });
  }
  wake(body: RigidBody): void {
    this.assertMember(body);
  }
  sleep(body: RigidBody): never {
    this.assertMember(body);
    throw new Error(
      "MuJoCo manual sleeping is not supported by the WASM bindings",
    );
  }
  readJoint(joint: Joint): JointReading {
    this.assertMember(joint);
    const binding = this.joints.get(joint);
    if (!binding) throw new Error("Missing MuJoCo joint binding");
    return binding.read(motionOf(this.api, this.native));
  }
  protected cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ) {
    refreshPoses(this.api, this.native);
    return raycast(
      this.api,
      this.native,
      origin,
      direction,
      maxDistance,
      options,
    );
  }
  private fingerprint(): Fingerprints {
    const members = [
      ...this.bodies.keys(),
      ...this.triggers,
      ...this.joints.keys(),
      ...this.added,
    ];
    return new Map(members.map((member) => [member, fingerprint(member)]));
  }
  /** Replaces the model with one compiled from the members, carrying `previous`'s state over. */
  private rebuild(
    previous: Simulation | undefined,
    fingerprints: Fingerprints,
  ): void {
    const bodies = new Map(this.bodies);
    const joints = new Map(this.joints);
    for (const object of this.added)
      if (object instanceof Joint) joints.set(object, new JointBinding(object));
      else
        // The body has no world yet, so this is the velocity it was given outside one.
        bodies.set(object, {
          pose: splitTransform(object.matrixWorld).pose,
          velocity: object.getVelocity(),
        });
    const triggers = [...this.triggers];
    const scales = lockScales([...bodies.keys(), ...triggers], this.scales);
    const next = compile(
      this.api,
      [...bodies.keys()],
      joints,
      triggers,
      this.options,
    );
    try {
      carryState(this.api, previous, next, joints.keys(), bodies);
      this.api.mj_forward(next.model, next.data);
    } catch (error) {
      rollback(error, [() => next.free()], "MuJoCo model replacement failed");
    }
    this.native.free();
    this.native = next;
    for (const [body, initial] of bodies) this.bodies.set(body, initial);
    for (const [joint, binding] of joints) this.joints.set(joint, binding);
    this.added.clear();
    this.fingerprints = fingerprints;
    this.scales = scales;
  }
}

function checkState(api: MainModule, sim: Simulation): void {
  if (
    !array(sim.data.qpos).every(Number.isFinite) ||
    !array(sim.data.qvel).every(Number.isFinite)
  )
    throw new Error("MuJoCo produced non-finite simulation state");
  // Unlike contact, warning is a borrowed vector owned by MjData; deleting it corrupts the WASM heap.
  const warnings = sim.data.warning;
  for (let i = 0; i < warnings.size(); i++) {
    const warning = warnings.get(i);
    if (!warning) throw new Error("Missing MuJoCo warning state");
    try {
      if (warning.number > 0)
        throw new Error(api.mju_warningText(i, warning.lastinfo));
    } finally {
      warning.delete();
    }
  }
}
