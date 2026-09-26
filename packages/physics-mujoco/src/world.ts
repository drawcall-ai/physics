import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  Trigger,
  SteppedWorld,
  assertLive,
  assertOwned,
  authoredVelocity,
  velocityAtPoint,
  setAuthoredVelocity,
  sceneJointReading,
  setWorldPose,
  splitTransform,
  cleanup,
  type PhysicsOptions,
  type PhysicsVelocity,
  type RaycastOptions,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { type Compiled } from "./model/compile.js";
import { array, at, pose, vector } from "./values.js";
import {
  bodyId,
  velocity,
  writeVelocity,
  writePose,
  synchronize,
  refreshPoses,
  validateState,
} from "./motion.js";
import { applyBodyForces, wrench } from "./forces.js";
import { Scene } from "./scene.js";
import { applyDrives } from "./drives.js";
import { sample, raycast } from "./queries.js";

export interface MujocoOptions extends PhysicsOptions {
  /**
   * MuJoCo's friction cone. Elliptic cones model friction faithfully; the default pyramids are what
   * MuJoCo ships, and they proved more robust for kinematic contact at small steps in this build.
   */
  frictionCone?: "pyramidal" | "elliptic";
  /**
   * MuJoCo's impratio: how stiff friction constraints are relative to normal ones. At the default 1
   * a static grip still creeps, because soft friction trades slip for force; raising it converges on
   * Coulomb friction without raising the limit at which contacts start to slide. It is defined for
   * elliptic cones, so any value above 1 needs `frictionCone: "elliptic"`.
   */
  frictionImpedanceRatio?: number;
  /** Browser bundlers can pass an emitted asset URL; Node resolves the packaged WASM automatically. */
  wasmUrl?: string;
}
export class MujocoWorld extends SteppedWorld {
  private readonly scene: Scene;
  private readonly pending: {
    body: RigidBody;
    run: (compiled: Compiled) => void;
  }[] = [];
  private readonly targets = new Map<RigidBody, Matrix4>();
  constructor(
    private readonly api: MainModule,
    options: MujocoOptions = {},
  ) {
    super(options);
    const frictionImpedanceRatio = options.frictionImpedanceRatio ?? 1;
    const frictionCone = options.frictionCone ?? "pyramidal";
    if (!Number.isFinite(frictionImpedanceRatio) || frictionImpedanceRatio < 1)
      throw new Error("frictionImpedanceRatio must be at least 1");
    if (frictionImpedanceRatio > 1 && frictionCone !== "elliptic")
      throw new Error(
        "frictionImpedanceRatio above 1 needs elliptic friction cones; set frictionCone",
      );
    this.scene = new Scene(api, {
      fixedDelta: this.fixedDelta,
      gravity: this.gravity,
      solverIterations: this.solverIterations ?? 50,
      frictionImpedanceRatio,
      frictionCone,
    });
  }
  register(object: RigidBody | Joint | Trigger): void {
    assertOwned(this, object);
    this.scene.register(object);
  }
  unregister(object: RigidBody | Joint | Trigger): void {
    if (!(object instanceof Joint)) this.interactions.remove(object);
    if (object instanceof RigidBody) this.targets.delete(object);
    cleanup(
      [() => this.scene.unregister(object), () => this.dispatch()],
      "Physics object removal failed",
    );
  }
  protected prepare(): Compiled {
    assertLive(this);
    return this.scene.prepare(this.time, (joint) => this.readJoint(joint));
  }
  protected step(): void {
    const compiled = this.prepare();
    for (const { body, run } of this.pending.splice(0))
      if (!body.disposed) run(compiled);
    let moved = false;
    for (const [body, matrix] of this.targets) {
      const id = compiled.targets.get(body);
      if (id === undefined) throw new Error("Missing MuJoCo kinematic target");
      if (writePose(this.api, compiled, id, matrix)) moved = true;
    }
    this.targets.clear();
    refreshPoses(this.api, compiled, moved);
    applyBodyForces(this.api, compiled, this.fixedDelta);
    applyDrives(
      this.api,
      compiled,
      this.scene.joints,
      (joint) => this.readJoint(joint),
      this.fixedDelta,
    );
    this.api.mj_step(compiled.model, compiled.data);
    this.api.mj_forward(compiled.model, compiled.data);
    validateState(this.api, compiled);
    array(compiled.data.qfrc_applied).fill(0);
    synchronize(compiled, setWorldPose);
    this.scene.trackAngles();
    refreshPoses(this.api, compiled);
    sample(this.api, compiled, this.interactions);
  }
  protected restore(): void {
    this.pending.length = 0;
    this.targets.clear();
    this.scene.reset();
  }
  protected disposeObjects(): void {
    this.pending.length = 0;
    this.targets.clear();
    this.scene.dispose();
  }
  protected free(): void {
    this.scene.free();
  }
  getVelocity(body: RigidBody): PhysicsVelocity {
    assertOwned(this, body);
    const id = this.scene.compiled?.bodies.get(body);
    return this.scene.compiled && id !== undefined
      ? velocity(this.api, this.scene.compiled, id)
      : authoredVelocity(body);
  }
  setVelocity(body: RigidBody, value: Partial<PhysicsVelocity>): void {
    assertOwned(this, body);
    const compiled = this.scene.compiled;
    const id = compiled?.bodies.get(body);
    if (!compiled || id === undefined) {
      setAuthoredVelocity(body, value);
      return;
    }
    writeVelocity(this.api, compiled, id, value);
    this.api.mj_forward(compiled.model, compiled.data);
  }
  teleport(body: RigidBody): void {
    assertOwned(this, body);
    this.targets.delete(body);
    const compiled = this.scene.compiled;
    const id = compiled?.bodies.get(body);
    if (!compiled || id === undefined) return;
    const { model, data } = compiled;
    const matrix = splitTransform(body.matrixWorld).pose;
    if (body.bodyType === "dynamic") {
      // The assembly moves rigidly, so the change of pose goes onto its root's free joint.
      const root = at(model.body_rootid, id);
      const base = [...compiled.bodies].find(([, index]) => index === root);
      if (base?.[0].bodyType !== "dynamic")
        throw new Error(
          "MuJoCo cannot teleport a body articulated to a static or kinematic base",
        );
      const delta = matrix
        .clone()
        .multiply(pose(data.xpos, data.xquat, id).invert());
      writePose(
        this.api,
        compiled,
        root,
        delta.multiply(pose(data.xpos, data.xquat, root)),
      );
    } else writePose(this.api, compiled, id, matrix);
    const target = compiled.targets.get(body);
    if (target !== undefined) writePose(this.api, compiled, target, matrix);
    this.api.mj_forward(model, data);
    synchronize(compiled, setWorldPose);
  }
  setKinematicTarget(body: RigidBody, matrix: Matrix4): void {
    assertOwned(this, body);
    this.targets.set(body, matrix.clone());
  }
  private push(
    body: RigidBody,
    value: Vector3,
    point: Vector3 | undefined,
    impulse: boolean,
  ): void {
    assertOwned(this, body);
    const force = value.clone(),
      atPoint = point?.clone();
    const run = (compiled: Compiled) => {
      const id = bodyId(compiled, body);
      wrench(
        this.api,
        compiled,
        id,
        force,
        new Vector3(),
        atPoint ?? vector(compiled.data.xipos, id * 3),
        impulse,
      );
    };
    if (impulse && this.scene.compiled?.bodies.has(body))
      run(this.scene.compiled);
    else this.pending.push({ body, run });
  }
  applyForce(body: RigidBody, force: Vector3, point?: Vector3): void {
    this.push(body, force, point, false);
  }
  applyImpulse(body: RigidBody, impulse: Vector3, point?: Vector3): void {
    this.push(body, impulse, point, true);
  }
  wake(body: RigidBody): void {
    assertOwned(this, body);
  }
  sleep(body: RigidBody): never {
    assertOwned(this, body);
    throw new Error(
      "MuJoCo manual sleeping is not supported by the WASM bindings",
    );
  }
  readJoint(joint: Joint) {
    assertOwned(this, joint);
    const record = this.scene.joints.get(joint);
    const reading = sceneJointReading(joint, record?.frames, (body, point) => {
      const compiled = this.scene.compiled;
      const id = compiled?.bodies.get(body);
      if (compiled && id !== undefined) {
        const value = velocity(this.api, compiled, id);
        const center = vector(compiled.data.xipos, id * 3);
        return value.linear.add(value.angular.cross(point.clone().sub(center)));
      }
      if (
        body.options.centerOfMass ||
        this.getVelocity(body).angular.lengthSq() === 0
      )
        return velocityAtPoint(body, point);
      const preview = this.scene.previewBody(body);
      try {
        const center = vector(preview.data.xipos, bodyId(preview, body) * 3);
        const value = this.getVelocity(body);
        return value.linear.add(value.angular.cross(point.clone().sub(center)));
      } finally {
        preview.free();
      }
    });
    if (record) reading.angle = record.angle;
    return reading;
  }
  raycast(
    origin: Vector3,
    direction: Vector3,
    maxDistance: number,
    options?: RaycastOptions,
  ) {
    assertLive(this);
    for (const body of options?.excludeBodies ?? []) assertOwned(this, body);
    const live = this.scene.matching();
    const compiled =
      live ?? this.scene.preview(this.time, (joint) => this.readJoint(joint));
    try {
      refreshPoses(this.api, compiled);
      return raycast(
        this.api,
        compiled,
        origin,
        direction,
        maxDistance,
        options,
      );
    } finally {
      if (!live) compiled.free();
    }
  }
}
