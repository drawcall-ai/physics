import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  AxisJoint,
  RevoluteJoint,
  DistanceJoint,
  RigidBody,
  authoredJointState,
  type Joint,
} from "@drawcall/physics";
import { Matrix4, Vector3 } from "three";
import { applyEffort, configure, joint } from "./joints.js";

type JointBinding = {
  target: Rapier.ImpulseJoint | undefined;
  frame0: Matrix4;
  frame1: Matrix4;
  settings: number;
  angle?: number;
  position?: number;
};

export class Constraints {
  readonly objects = new Set<Joint>();
  private readonly bindings = new Map<Joint, JointBinding>();
  private readonly efforts = new Map<AxisJoint, number>();
  private anchor: Rapier.RigidBody | undefined;
  constructor(
    private readonly api: typeof Rapier,
    private readonly backend: Rapier.World,
    private readonly getBody: (object: RigidBody) => Rapier.RigidBody,
  ) {}

  remove(object: Joint): void {
    this.objects.delete(object);
    if (object instanceof AxisJoint) this.efforts.delete(object);
    const target = this.bindings.get(object)?.target;
    if (target) this.backend.removeImpulseJoint(target, true);
    this.bindings.delete(object);
  }
  setEffort(object: AxisJoint, value: number): void {
    if (!Number.isFinite(value)) throw new Error("Joint effort must be finite");
    if (!object.enabled || value === 0) this.efforts.delete(object);
    else this.efforts.set(object, value);
  }
  applyEfforts(): void {
    for (const [object, value] of this.efforts) {
      const binding = this.bindings.get(object);
      if (binding?.target && object.enabled)
        applyEffort(object, binding.target, value);
    }
  }
  clearEfforts(): void {
    this.efforts.clear();
  }
  getState(object: Joint) {
    const binding = this.bindings.get(object);
    const state = this.readState(object, binding);
    if (
      object instanceof RevoluteJoint &&
      binding &&
      "position" in state &&
      binding.position !== undefined
    )
      return { ...state, position: binding.position };
    return state;
  }
  private readState(object: Joint, binding?: JointBinding) {
    if (!binding) return authoredJointState(object);
    const first = object.options.body0
      ? this.getBody(object.options.body0).worldCom()
      : new Vector3();
    const second = this.getBody(object.options.body1).worldCom();
    return authoredJointState(
      object,
      [binding.frame0, binding.frame1],
      [new Vector3().copy(first), new Vector3().copy(second)],
    );
  }
  sampleAngles(rebase = false, body?: RigidBody): void {
    for (const [object, binding] of this.bindings) {
      if (!(object instanceof RevoluteJoint)) continue;
      if (
        body &&
        object.options.body0 !== body &&
        object.options.body1 !== body
      )
        continue;
      const state = this.readState(object, binding);
      if (!("position" in state)) throw new Error("Expected revolute state");
      const angle = state.position;
      if (
        rebase ||
        binding.angle === undefined ||
        binding.position === undefined
      )
        binding.position = angle;
      else
        binding.position += Math.atan2(
          Math.sin(angle - binding.angle),
          Math.cos(angle - binding.angle),
        );
      binding.angle = angle;
    }
  }
  refresh(pendingOnly: boolean): void {
    for (const object of this.objects) {
      if (pendingOnly && this.bindings.has(object)) continue;
      object.validate();
      let binding = this.bindings.get(object);
      if (!binding) {
        binding = {
          frame0: object.getFrame(0, new Matrix4()),
          frame1: object.getFrame(1, new Matrix4()),
          target: undefined,
          settings: -1,
        };
        this.bindings.set(object, binding);
        if (object instanceof RevoluteJoint) {
          const state = this.readState(object, binding);
          if (!("position" in state))
            throw new Error("Expected revolute state");
          binding.angle = state.position;
          binding.position = state.position;
        }
      }
      if (binding.settings === object.settingsVersion) continue;
      if (!object.enabled) {
        if (binding.target)
          this.backend.removeImpulseJoint(binding.target, true);
        binding.target = undefined;
        if (object instanceof AxisJoint) this.efforts.delete(object);
        binding.settings = object.settingsVersion;
        continue;
      }
      if (!binding.target || object instanceof DistanceJoint) {
        if (!object.options.body0 && !this.anchor)
          this.anchor = this.backend.createRigidBody(
            this.api.RigidBodyDesc.fixed(),
          );
        const first = object.options.body0
          ? this.getBody(object.options.body0)
          : this.anchor;
        if (!first) throw new Error("Missing world anchor");
        const replacement = joint(
          this.api,
          this.backend,
          object,
          binding.frame0,
          binding.frame1,
          first,
          this.getBody(object.options.body1),
        );
        if (binding.target)
          this.backend.removeImpulseJoint(binding.target, true);
        binding.target = replacement;
      }
      configure(this.api, object, binding.target);
      binding.settings = object.settingsVersion;
    }
  }
}
