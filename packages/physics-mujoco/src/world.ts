import type { MainModule } from "@mujoco/mujoco";
import {
  RigidBody,
  Joint,
  Trigger,
  assertLive,
  assertOwned,
  authoredVelocity,
  setAuthoredVelocity,
  authoredJointReading,
  setWorldPose,
  clearDefaultWorld,
  cleanup,
  wrapAngle,
  type PhysicsWorld,
  type PhysicsOptions,
  type PhysicsVelocity,
  type RaycastOptions,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { type Compiled } from "./model.js";
import { array, vector } from "./values.js";
import {
  bodyId,
  freeJoint,
  velocity,
  writeVelocity,
  writePose,
  wrench,
  synchronize,
  refreshPoses,
  validateState,
} from "./motion.js";
import { Scene } from "./scene.js";
import { applyDrives } from "./drives.js";
import { sample, raycast } from "./queries.js";
import { Interactions } from "@drawcall/physics";

export interface MujocoOptions extends PhysicsOptions {
  solverIterations?: number;
  /** Browser bundlers can pass an emitted asset URL; Node resolves the packaged WASM automatically. */
  wasmUrl?: string;
}
export class MujocoWorld implements PhysicsWorld {
  readonly fixedDelta: number;
  private readonly maxSubsteps: number;
  private readonly scene: Scene;
  private readonly before = new Set<(delta: number) => void>();
  private readonly after = new Set<(delta: number) => void>();
  private readonly pending: {
    body: RigidBody;
    run: (compiled: Compiled) => void;
  }[] = [];
  private readonly targets = new Map<RigidBody, Matrix4>();
  private readonly interactions = new Interactions();
  private elapsed = 0;
  private completed = 0;
  private updating = false;
  private isDisposed = false;
  get disposed(): boolean {
    return this.isDisposed;
  }
  get time(): number {
    return this.completed;
  }
  constructor(
    private readonly api: MainModule,
    options: MujocoOptions = {},
  ) {
    this.fixedDelta = options.fixedDelta ?? 1 / 60;
    this.maxSubsteps = options.maxSubsteps ?? 5;
    const gravity = [...(options.gravity ?? [0, -9.81, 0])];
    const solverIterations = options.solverIterations ?? 50;
    if (!Number.isFinite(this.fixedDelta) || this.fixedDelta <= 0)
      throw new Error("fixedDelta must be positive and finite");
    if (!Number.isInteger(this.maxSubsteps) || this.maxSubsteps < 1)
      throw new Error("maxSubsteps must be a positive integer");
    if (!gravity.every(Number.isFinite))
      throw new Error("Gravity must be finite");
    if (!Number.isInteger(solverIterations) || solverIterations < 1)
      throw new Error("solverIterations must be a positive integer");
    this.scene = new Scene(api, {
      fixedDelta: this.fixedDelta,
      gravity,
      solverIterations,
    });
  }
  register(object: RigidBody | Joint | Trigger): void {
    assertOwned(this, object);
    if (object instanceof RigidBody) this.scene.objects.add(object);
    else if (object instanceof Joint) this.scene.jointObjects.add(object);
    else this.scene.triggers.add(object);
  }
  unregister(object: RigidBody | Joint | Trigger): void {
    this.scene.key = "";
    if (object instanceof Joint) {
      this.scene.jointObjects.delete(object);
      this.scene.joints.delete(object);
      return;
    }
    this.interactions.remove(object);
    if (object instanceof Trigger) this.scene.triggers.delete(object);
    else {
      this.scene.objects.delete(object);
      this.scene.bodies.delete(object);
      this.targets.delete(object);
      cleanup(
        [...this.scene.jointObjects]
          .filter((j) => j.connects(object))
          .map((j) => () => j.dispose()),
        "Joint disposal failed",
      );
    }
    object.traverse((child) => this.scene.scales.delete(child));
    this.interactions.dispatch();
  }
  private prepare(): Compiled {
    assertLive(this);
    return this.scene.prepare(this.completed, (joint) => this.readJoint(joint));
  }
  update(delta: number): void {
    assertLive(this);
    this.assertIdle();
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("delta must be finite and nonnegative");
    this.updating = true;
    try {
      this.prepare();
      this.elapsed = Math.min(
        this.elapsed + delta,
        this.fixedDelta * this.maxSubsteps,
      );
      while (!this.disposed && this.elapsed >= this.fixedDelta) {
        for (const callback of this.before) {
          callback(this.fixedDelta);
          if (this.disposed) return;
        }
        const compiled = this.prepare();
        for (const { body, run } of this.pending.splice(0))
          if (!body.disposed) run(compiled);
        for (const [body, matrix] of this.targets)
          writePose(compiled, bodyId(compiled, body), matrix);
        this.targets.clear();
        refreshPoses(this.api, compiled);
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
        this.elapsed -= this.fixedDelta;
        this.completed += this.fixedDelta;
        for (const [joint, record] of this.scene.joints) {
          const sampled = authoredJointReading(
            joint,
            record.frames,
            () => new Vector3(),
          ).angle;
          record.angle += wrapAngle(sampled - record.sampled);
          record.sampled = sampled;
        }
        refreshPoses(this.api, compiled);
        sample(this.api, compiled, this.interactions);
        this.interactions.dispatch();
        for (const callback of this.after) {
          if (this.disposed) return;
          callback(this.fixedDelta);
        }
      }
    } finally {
      this.updating = false;
      if (this.disposed) this.free();
    }
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
    if (body.bodyType !== "dynamic") {
      setAuthoredVelocity(body, value);
      return;
    }
    const id = this.scene.compiled?.bodies.get(body);
    if (!this.scene.compiled || id === undefined) {
      setAuthoredVelocity(body, value);
      return;
    }
    writeVelocity(this.scene.compiled, id, value);
    this.api.mj_forward(this.scene.compiled.model, this.scene.compiled.data);
  }
  teleport(body: RigidBody, matrix: Matrix4): void {
    assertOwned(this, body);
    body.validate();
    const id = this.scene.compiled?.bodies.get(body);
    if (this.scene.compiled && id !== undefined) {
      if (body.bodyType === "dynamic") freeJoint(this.scene.compiled, id);
      writePose(this.scene.compiled, id, matrix);
      this.api.mj_forward(this.scene.compiled.model, this.scene.compiled.data);
      synchronize(this.scene.compiled, setWorldPose);
    }
    setWorldPose(body, matrix);
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
    if (body.bodyType !== "dynamic") return;
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
    const reading = authoredJointReading(
      joint,
      record?.frames,
      (body, point) => {
        const value = this.getVelocity(body);
        const id = this.scene.compiled?.bodies.get(body);
        const center =
          this.scene.compiled && id !== undefined
            ? vector(this.scene.compiled.data.xipos, id * 3)
            : new Vector3().setFromMatrixPosition(body.matrixWorld);
        return value.linear.add(value.angular.cross(point.clone().sub(center)));
      },
    );
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
    const compiled = this.scene.preview(this.time, (joint) =>
      this.readJoint(joint),
    );
    try {
      return raycast(
        this.api,
        compiled,
        origin,
        direction,
        maxDistance,
        options,
      );
    } finally {
      compiled.free();
    }
  }
  getOverlappingBodies(trigger: Trigger): RigidBody[] {
    assertOwned(this, trigger);
    return this.interactions.bodies(trigger);
  }
  onBeforeStep(callback: (delta: number) => void): () => void {
    assertLive(this);
    this.before.add(callback);
    return () => this.before.delete(callback);
  }
  onAfterStep(callback: (delta: number) => void): () => void {
    assertLive(this);
    this.after.add(callback);
    return () => this.after.delete(callback);
  }
  reset(): void {
    assertLive(this);
    this.assertIdle();
    this.interactions.clear();
    this.pending.length = 0;
    this.targets.clear();
    this.free();
    for (const [body, record] of this.scene.bodies) {
      setWorldPose(body, record.initialPose);
      setAuthoredVelocity(body, record.initialVelocity);
    }
    for (const [joint, record] of this.scene.joints) {
      record.angle = authoredJointReading(
        joint,
        record.frames,
        () => new Vector3(),
      ).angle;
      record.sampled = record.angle;
    }
    this.elapsed = 0;
    this.completed = 0;
  }
  dispose(): void {
    if (this.disposed) return;
    this.isDisposed = true;
    this.interactions.clear();
    cleanup(
      [
        ...this.scene.triggers,
        ...this.scene.jointObjects,
        ...this.scene.objects,
      ]
        .map((object) => () => object.dispose())
        .concat(() => {
          this.before.clear();
          this.after.clear();
          this.pending.length = 0;
          this.targets.clear();
          this.scene.scales.clear();
          clearDefaultWorld(this);
          if (!this.updating) this.free();
        }),
      "MuJoCo world disposal failed",
    );
  }
  private free(): void {
    this.scene.free();
  }
  private assertIdle(): void {
    if (this.updating || this.interactions.dispatching)
      throw new Error(
        "Cannot update or reset a world during a simulation callback",
      );
  }
}
