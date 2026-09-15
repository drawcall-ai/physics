import { AxisJoint } from "./joints.js";

export interface JointMotorOptions {
  readonly joint: AxisJoint;
  readonly stiffness?: number;
  readonly damping?: number;
  readonly maxForce?: number;
  readonly model?: "force" | "acceleration";
}
export interface JointMotorTarget {
  readonly position: number;
  readonly velocity: number;
}
const motors = new WeakMap<AxisJoint, JointMotor>();

/** One native actuator per axis joint. No target means no actuation. */
export class JointMotor {
  readonly #options: JointMotorOptions;
  #target?: JointMotorTarget;
  #enabled = true;
  #disposed = false;
  #version = 0;
  constructor(options: JointMotorOptions) {
    if (!(options.joint instanceof AxisJoint))
      throw new Error("Joint motors require a single-axis joint");
    if (options.joint.disposed)
      throw new Error("Cannot motorize a disposed joint");
    if (motors.has(options.joint)) throw new Error("Joint already has a motor");
    for (const value of [
      options.stiffness,
      options.damping,
      options.maxForce,
    ]) {
      if (value !== undefined && (!Number.isFinite(value) || value < 0))
        throw new Error(
          "Motor gains and maximum effort must be finite and nonnegative",
        );
    }
    if (
      options.model !== undefined &&
      options.model !== "force" &&
      options.model !== "acceleration"
    )
      throw new Error("Unknown motor model");
    this.#options = Object.freeze({ ...options });
    motors.set(options.joint, this);
  }
  get options(): JointMotorOptions {
    return this.#options;
  }
  get target(): JointMotorTarget | undefined {
    return this.#target;
  }
  get enabled(): boolean {
    return this.#enabled;
  }
  get active(): boolean {
    return !this.disposed && this.enabled && this.target !== undefined;
  }
  get disposed(): boolean {
    return this.#disposed;
  }
  get settingsVersion(): number {
    return this.#version;
  }
  setTarget(value: Partial<JointMotorTarget>): this {
    this.assertLive();
    if (value.position === undefined && value.velocity === undefined)
      throw new Error("Motor target requires position or velocity");
    if (
      [value.position, value.velocity].some(
        (v) => v !== undefined && !Number.isFinite(v),
      )
    )
      throw new Error("Motor targets must be finite");
    this.#target = Object.freeze({
      position: value.position ?? 0,
      velocity: value.velocity ?? 0,
    });
    this.#version++;
    return this;
  }
  setEnabled(value: boolean): this {
    this.assertLive();
    if (typeof value !== "boolean")
      throw new Error("Motor enabled must be boolean");
    this.#enabled = value;
    this.#version++;
    return this;
  }
  dispose(): void {
    if (this.disposed) return;
    motors.delete(this.options.joint);
    this.#disposed = true;
    this.#version++;
  }
  private assertLive(): void {
    if (this.disposed || this.options.joint.disposed)
      throw new Error("Joint motor has been disposed");
  }
}

/** Backend and joint integration; motor ownership changes only through its constructor/disposal. */
export function getJointMotor(joint: AxisJoint): JointMotor | undefined {
  return motors.get(joint);
}
