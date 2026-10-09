import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  PhysicsWorld,
  splitTransform,
  Trigger,
  type JointReading,
  type PhysicsVelocity,
  type RaycastOptions,
} from "@drawcall/physics";
import {
  authoredVelocity,
  rollback,
  setWorldPose,
  treeJoint,
  JointBinding,
  type Decomposes,
  type Initial,
} from "@drawcall/physics/backend";
import type { BufferGeometry, Matrix4, Object3D, Vector3 } from "three";
import { heightfield } from "./model/heightfield.js";
import {
  compile,
  type ModelOptions,
  type Simulation,
} from "./model/compile.js";
import { carryState } from "./model/state.js";
import { modelOptions, type MujocoWorldOptions } from "./options.js";
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

/** MuJoCo collides only convex shapes: every triangle mesh but a static height grid. */
const decomposes: Decomposes = (body: RigidBody, geometry: BufferGeometry) =>
  body.bodyType !== "static" || !heightfield(geometry, "grid");

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
  protected add(object: RigidBody | Joint | Trigger): void {
    if (object instanceof RigidBody || object instanceof Joint)
      this.joining.add(object);
    else this.triggers.add(object);
  }
  /**
   * The model holds a member that left until the next rebuild, which can fail. Forgetting it
   * there now means no rebuild carries its simulated state over, even when it rejoins.
   */
  protected remove(object: RigidBody | Joint | Trigger): void {
    this.compiled = undefined;
    if (object instanceof Joint) {
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
    let moved = false;
    for (const [body, matrix] of this.targets) {
      const id = simulation.targets.get(body);
      if (id === undefined) throw new Error("Missing MuJoCo kinematic target");
      if (writePose(api, simulation, id, matrix)) moved = true;
    }
    this.targets.clear();
    refreshPoses(api, simulation, moved);
    damp(api, simulation, this.fixedDelta);
    applyDrives(api, simulation, this.joints, this.fixedDelta);
    // Split step: the state already holds step1 results for this pose (the last mj_step1 or any
    // mj_forward), so collision and the solve run once per step rather than in mj_step and mj_forward.
    api.mj_step2(simulation.model, simulation.data);
    api.mj_step1(simulation.model, simulation.data);
    checkState(api, simulation);
    array(simulation.data.qfrc_applied).fill(0);
    writeBack(simulation);
    for (const binding of this.joints.values()) binding.track();
    refreshPoses(api, simulation);
    sampleInteractions(api, simulation, this.interactions);
  }
  protected restore(): void {
    this.forces.length = 0;
    this.targets.clear();
    for (const [body, { pose }] of this.bodies) setWorldPose(body, pose);
    for (const binding of this.joints.values()) binding.rebase();
    // A rebuild without the previous model starts every body from its authored motion.
    this.rebuild(undefined, this.fingerprints());
  }
  protected free(): void {
    this.simulation.free();
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
    const members = [
      ...this.bodies.keys(),
      ...this.triggers,
      ...this.joints.keys(),
      ...this.joining,
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
      else
        bodies.set(object, {
          pose: splitTransform(object.matrixWorld).pose,
          velocity: authoredVelocity(object),
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
    this.simulation.free();
    this.simulation = next;
    for (const [body, initial] of bodies) this.bodies.set(body, initial);
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
