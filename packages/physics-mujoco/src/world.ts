import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  SoftBody,
  SoftAttachment,
  Joint,
  PhysicsWorld,
  splitTransform,
  Trigger,
  type JointReading,
  type PhysicsVelocity,
  type RaycastOptions,
} from "@drawcall/physics";
import {
  rollback,
  setWorldPose,
  treeJoint,
  JointBinding,
  type Member,
} from "@drawcall/physics/backend";
import { Vector3, type Matrix4, type Object3D } from "three";
import { decomposes } from "./model/shapes.js";
import {
  compile,
  type ModelOptions,
  type Simulation,
} from "./model/compile.js";
import { carryState } from "./model/state.js";
import { modelOptions, type MujocoWorldOptions } from "./options.js";
import { softMesh, type SoftMesh } from "./soft/mesh.js";
import { attachment, type Attachment } from "./soft/model.js";
import {
  carrySoft,
  dampSoft,
  softPositions,
  softLoad,
  writeSoftGeometry,
} from "./soft/state.js";
import { array } from "./heap.js";
import {
  bodyId,
  motionOf,
  velocity,
  writeVelocity,
  writePose,
  writeBack,
  refreshPoses,
  teleport,
} from "./rigid/body.js";
import { damp, wrench } from "./rigid/force.js";
import { applyDrives } from "./drive.js";
import { raycast } from "./query.js";
import { sampleContacts, sampleOverlaps } from "./interactions.js";
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
  /** Each body's pose when a model first held it, which reset returns it to. */
  private readonly bodies = new Map<RigidBody, Matrix4>();
  private readonly joints = new Map<Joint, JointBinding>();
  private readonly triggers = new Set<Trigger>();
  private readonly soft = new Map<SoftBody, SoftMesh>();
  private readonly attachments = new Map<SoftAttachment, Attachment>();
  private softForces: { body: SoftBody; force: Vector3; point?: Vector3 }[] =
    [];
  /** Bodies and joints no model has held yet; a successful rebuild binds them. */
  private readonly joining = new Set<RigidBody | Joint>();
  private simulation: Simulation;
  /** The fingerprints the model was compiled from; undefined once a member left it. */
  private compiled: Fingerprints | undefined = new Map();
  private scales = new Map<Object3D, Vector3>();
  // Forces and targets wait for the step: a rebuild while refreshing would discard their writes.
  private forces: { body: RigidBody; force: Vector3; point?: Vector3 }[] = [];
  private readonly targets = new Map<RigidBody, Matrix4>();
  constructor(
    private readonly api: MainModule,
    options: MujocoWorldOptions,
  ) {
    super(options, decomposes);
    this.options = modelOptions(options, {
      fixedDelta: this.fixedDelta,
      gravity: this.gravity,
      solverIterations: this.solverIterations ?? 50,
    });
    this.simulation = compile(api, [], this.joints, [], this.options);
  }
  protected add(object: Member): void {
    if (object instanceof SoftBody) {
      this.soft.set(object, softMesh(object));
    } else if (object instanceof SoftAttachment) {
      const mesh = this.soft.get(object.options.body);
      if (!mesh) throw new Error("Soft attachment requires a joined body");
      const positions = this.simulation.soft.has(object.options.body)
        ? softPositions(this.simulation, object.options.body)
        : mesh.rest;
      this.attachments.set(object, attachment(object, mesh, positions));
    } else if (object instanceof RigidBody || object instanceof Joint)
      this.joining.add(object);
    else this.triggers.add(object);
  }
  /**
   * The model holds a member that left until the next rebuild, which can fail. Forgetting it
   * there now means no rebuild carries its simulated state over, even when it rejoins.
   */
  protected remove(object: Member): void {
    this.compiled = undefined;
    if (object instanceof SoftAttachment) {
      this.attachments.delete(object);
    } else if (object instanceof SoftBody) {
      this.soft.delete(object);
      this.simulation.soft.delete(object);
      this.softForces = this.softForces.filter(({ body }) => body !== object);
    } else if (object instanceof Joint) {
      this.joining.delete(object);
      this.joints.delete(object);
    } else if (object instanceof Trigger) {
      this.triggers.delete(object);
    } else {
      this.joining.delete(object);
      this.bodies.delete(object);
      this.simulation.bodies.delete(object);
      this.simulation.targets.delete(object);
      this.targets.delete(object);
      this.forces = this.forces.filter(({ body }) => body !== object);
    }
  }
  protected prepare(): void {
    const current = this.fingerprints();
    if (this.compiled && sameFingerprints(current, this.compiled)) return;
    this.rebuild(this.simulation, current);
  }
  protected step(): void {
    const { api, simulation } = this;
    for (const { body, force, point } of this.forces.splice(0))
      wrench(api, simulation, bodyId(simulation, body), force, { point });
    for (const { body, force, point } of this.softForces.splice(0))
      softLoad(api, simulation, body, force, point, false);
    let moved = false;
    for (const [body, matrix] of this.targets) {
      const id = simulation.targets.get(body);
      if (id === undefined) throw new Error("Missing MuJoCo kinematic target");
      if (writePose(api, simulation, id, matrix)) moved = true;
    }
    this.targets.clear();
    refreshPoses(api, simulation, moved);
    damp(api, simulation, this.fixedDelta);
    dampSoft(simulation, this.fixedDelta);
    applyDrives(api, simulation, this.joints, this.fixedDelta);
    // Split step: the state already holds step1 results for this pose (the last mj_step1 or any
    // mj_forward), so collision and the solve run once per step rather than in mj_step and mj_forward.
    api.mj_step2(simulation.model, simulation.data);
    const contacts = sampleContacts(api, simulation);
    api.mj_step1(simulation.model, simulation.data);
    checkState(api, simulation);
    array(simulation.data.qfrc_applied).fill(0);
    writeBack(simulation);
    writeSoftGeometry(simulation);
    for (const binding of this.joints.values()) binding.track();
    refreshPoses(api, simulation);
    this.interactions.replace(sampleOverlaps(api, simulation), contacts);
  }
  protected restore(): void {
    this.forces.length = 0;
    this.softForces.length = 0;
    this.targets.clear();
    for (const [body, pose] of this.bodies) setWorldPose(body, pose);
    for (const binding of this.joints.values()) binding.rebase();
    // A rebuild without the previous model starts every body from its initial velocity.
    this.rebuild(undefined, this.fingerprints());
  }
  protected free(): void {
    this.simulation.free();
  }
  protected readSoftPositions(body: SoftBody): Float32Array {
    return softPositions(this.simulation, body);
  }
  protected writeSoftForce(
    body: SoftBody,
    force: Vector3,
    point?: Vector3,
  ): void {
    this.softForces.push({ body, force: force.clone(), point: point?.clone() });
  }
  protected writeSoftImpulse(
    body: SoftBody,
    impulse: Vector3,
    point?: Vector3,
  ): void {
    softLoad(this.api, this.simulation, body, impulse, point, true);
  }
  protected readVelocity(body: RigidBody): PhysicsVelocity {
    return velocity(this.api, this.simulation, bodyId(this.simulation, body));
  }
  protected writeVelocity(
    body: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void {
    writeVelocity(
      this.api,
      this.simulation,
      bodyId(this.simulation, body),
      value,
    );
    this.api.mj_forward(this.simulation.model, this.simulation.data);
  }
  protected writePoses(bodies: ReadonlySet<RigidBody>): void {
    const anchored = [...this.joints.keys()].some(
      (joint) =>
        treeJoint(joint) &&
        bodies.has(joint.options.body1) &&
        !(joint.options.body0 && bodies.has(joint.options.body0)),
    );
    if (anchored)
      throw new Error(
        "MuJoCo cannot teleport a body articulated to the world or a static or kinematic base",
      );
    for (const body of bodies) this.targets.delete(body);
    teleport(this.api, this.simulation, bodies);
    for (const binding of this.joints.values()) binding.rebaseIfSplit(bodies);
  }
  protected writeTarget(body: RigidBody, pose: Matrix4): void {
    this.targets.set(body, pose.clone());
  }
  protected writeForce(body: RigidBody, force: Vector3, point?: Vector3): void {
    this.forces.push({ body, force: force.clone(), point: point?.clone() });
  }
  protected writeImpulse(
    body: RigidBody,
    impulse: Vector3,
    point?: Vector3,
  ): void {
    const { api, simulation } = this;
    wrench(api, simulation, bodyId(simulation, body), impulse, {
      point,
      impulse: true,
    });
  }
  protected writeSleeping(_body: RigidBody, sleeping: boolean): void {
    if (sleeping)
      throw new Error(
        "MuJoCo manual sleeping is not supported by the WASM bindings",
      );
  }
  protected jointReading(joint: Joint): JointReading {
    const binding = this.joints.get(joint);
    if (!binding) throw new Error("Missing MuJoCo joint binding");
    return binding.read(motionOf(this.api, this.simulation));
  }
  protected cast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options: RaycastOptions,
  ) {
    refreshPoses(this.api, this.simulation);
    return raycast(
      this.api,
      this.simulation,
      origin,
      direction,
      maxDistance,
      options,
    );
  }
  private fingerprints(): Fingerprints {
    for (const [body, mesh] of this.soft) {
      body.validate();
      const scale = new Vector3().setFromMatrixScale(body.matrixWorld).x;
      if (Math.abs(scale - mesh.scale) > 1e-6)
        throw new Error("Soft body scale cannot change after joining a world");
    }
    const members = [
      ...this.bodies.keys(),
      ...this.triggers,
      ...this.joints.keys(),
      ...this.joining,
      ...this.soft.keys(),
      ...this.attachments.keys(),
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
    for (const object of this.joining)
      if (object instanceof Joint) joints.set(object, new JointBinding(object));
      else bodies.set(object, splitTransform(object.matrixWorld).pose);
    const triggers = [...this.triggers];
    const scales = lockScales([...bodies.keys(), ...triggers], this.scales);
    const next = compile(
      this.api,
      [...bodies.keys()],
      joints,
      triggers,
      this.options,
      this.soft,
      this.attachments.values(),
    );
    try {
      carryState(this.api, previous, next, joints);
      carrySoft(previous, next);
      this.api.mj_forward(next.model, next.data);
      checkState(this.api, next);
    } catch (error) {
      rollback(error, [() => next.free()], "MuJoCo model replacement failed");
    }
    this.simulation.free();
    this.simulation = next;
    writeSoftGeometry(next);
    for (const [body, pose] of bodies) this.bodies.set(body, pose);
    for (const [joint, binding] of joints) this.joints.set(joint, binding);
    this.joining.clear();
    this.compiled = fingerprints;
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
