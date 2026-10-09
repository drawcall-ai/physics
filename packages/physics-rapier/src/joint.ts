import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  AxisJoint,
  DistanceJoint,
  FixedJoint,
  GenericJoint,
  PrismaticJoint,
  RevoluteJoint,
  SphericalJoint,
  jointDofs,
  type Joint,
} from "@drawcall/physics";
import { axisVector, JointBinding } from "@drawcall/physics/backend";
import { Quaternion, Vector3 } from "three";
import { configureDrives, rapierDof, validateDrives } from "./drive.js";

/** A joint binding with the Rapier joint that simulates it. */
export class RapierJointBinding extends JointBinding {
  /** Absent while the joint is disabled. */
  native: Rapier.ImpulseJoint | undefined = undefined;
  /** The joint version the native joint reflects; it counts drive changes too. */
  version = -1;

  constructor(
    joint: Joint,
    readonly bodies: readonly [Rapier.RigidBody, Rapier.RigidBody],
  ) {
    super(joint);
  }
}

/** Reconciles the native joint with the authored settings and drives. */
export function prepareJoint(
  api: typeof Rapier,
  native: Rapier.World,
  binding: RapierJointBinding,
): void {
  const { joint } = binding;
  joint.validate();
  assertReachableTarget(binding);
  if (binding.version === joint.version) return;
  if (joint.enabled) {
    validateDrives(joint);
    binding.native ??= createJoint(api, native, binding);
    binding.native.setContactsEnabled(joint.collideConnected);
    configureDrives(api, native, joint, binding.native);
    binding.native.body1().wakeUp();
    binding.native.body2().wakeUp();
  } else if (binding.native) {
    native.removeImpulseJoint(binding.native, true);
    binding.native = undefined;
  }
  binding.version = joint.version;
}

/** Rapier's shortest-arc motor cannot aim half a turn or more away from the current angle. */
function assertReachableTarget(binding: RapierJointBinding): void {
  const { joint } = binding;
  if (!(joint instanceof RevoluteJoint) || !joint.enabled) return;
  const drive = joint.drive;
  if (!drive?.target || (drive.options.stiffness ?? 0) <= 0) return;
  if (Math.abs(drive.target.position - binding.angle) >= Math.PI)
    throw new Error(
      "Revolute drive position must remain within pi radians of the current continuous angle; use intermediate targets for longer moves",
    );
}

/** Creates the native joint with its anchors and limits, which never change afterwards. */
function createJoint(
  api: typeof Rapier,
  native: Rapier.World,
  binding: RapierJointBinding,
): Rapier.ImpulseJoint {
  const object = binding.joint;
  const [frame0, frame1] = binding.frames;
  const a = new Vector3().setFromMatrixPosition(frame0);
  const b = new Vector3().setFromMatrixPosition(frame1);
  const rotation0 = new Quaternion().setFromRotationMatrix(frame0);
  const rotation1 = new Quaternion().setFromRotationMatrix(frame1);
  const data = jointData(api, object, a, rotation0, b, rotation1);
  const joint = native.createImpulseJoint(data, ...binding.bodies, true);
  const raw = native.impulseJoints.raw;
  if (object instanceof DistanceJoint && Number.isFinite(object.limits[1]))
    raw.jointSetLimits(
      joint.handle,
      rapierDof(api.JointAxis, "transX"),
      0,
      object.limits[1],
    );
  if (object instanceof AxisJoint) {
    const rotation = new Quaternion().setFromUnitVectors(
      new Vector3(1, 0, 0),
      axisVector(object.options.axis),
    );
    joint.setLocalFrame1(a, rotation0.clone().multiply(rotation));
    joint.setLocalFrame2(b, rotation1.clone().multiply(rotation));
    if (!(joint instanceof api.UnitImpulseJoint))
      throw new Error("Expected a Rapier unit joint");
    if (object.options.limits) joint.setLimits(...object.options.limits);
  }
  if (object instanceof GenericJoint) {
    joint.setLocalFrame1(a, rotation0);
    joint.setLocalFrame2(b, rotation1);
    for (const axis of jointDofs) {
      const motion = object.dofs[axis];
      if (typeof motion !== "string")
        raw.jointSetLimits(
          joint.handle,
          rapierDof(api.JointAxis, axis),
          motion[0],
          motion[1],
        );
    }
  }
  return joint;
}

/** Rapier's description of the joint type; axis joints are built along X and reframed afterwards. */
function jointData(
  api: typeof Rapier,
  object: Joint,
  a: Vector3,
  rotation0: Quaternion,
  b: Vector3,
  rotation1: Quaternion,
): Rapier.JointData {
  const x = new Vector3(1, 0, 0);
  if (object instanceof FixedJoint)
    return api.JointData.fixed(a, rotation0, b, rotation1);
  if (object instanceof SphericalJoint) return api.JointData.spherical(a, b);
  if (object instanceof DistanceJoint) {
    if (object.limits[0] !== 0)
      throw new Error(
        "Rapier distance joints only support a zero minimum distance",
      );
    // A spring joint is Rapier's driven distance constraint; its rope limit is set after creation.
    return api.JointData.spring(0, 0, 0, a, b);
  }
  if (object instanceof RevoluteJoint) return api.JointData.revolute(a, b, x);
  if (object instanceof PrismaticJoint) return api.JointData.prismatic(a, b, x);
  if (object instanceof GenericJoint) {
    let locked = 0;
    for (const axis of jointDofs)
      if (object.dofs[axis] === "locked")
        locked |= rapierDof(api.JointAxesMask, axis);
    return api.JointData.generic(a, b, x, locked);
  }
  throw new Error(`Unsupported Rapier joint: ${object.type}`);
}
