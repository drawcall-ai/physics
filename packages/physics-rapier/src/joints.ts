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
  axisVector,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import {
  configureDrives,
  driveStates,
  rapierDof,
  sameDrives,
  type DriveState,
} from "./drives.js";
import { readBinding, rebaseAngle } from "./reading.js";

export interface JointBinding {
  /** Absent while the joint is disabled. */
  joint: Rapier.ImpulseJoint | undefined;
  /** Body-local anchors, captured at first preparation. */
  frames: readonly [Matrix4, Matrix4];
  bodies: readonly [Rapier.RigidBody, Rapier.RigidBody];
  settingsVersion: number;
  driveStates: DriveState[];
  /** Turn counting: the last wrapped sample and the continuous position. */
  angle?: { sampled: number; continuous: number };
}

/** Creates or reconciles the backend joint with the authored settings and drives. */
export function prepareJoint(
  api: typeof Rapier,
  backend: Rapier.World,
  object: Joint,
  body0: Rapier.RigidBody,
  body1: Rapier.RigidBody,
  previous?: JointBinding,
): JointBinding {
  object.validate();
  const binding =
    previous ??
    ({
      frames: [
        object.getFrame(0, new Matrix4()),
        object.getFrame(1, new Matrix4()),
      ],
      bodies: [body0, body1],
      joint: undefined,
      settingsVersion: -1,
      driveStates: [],
    } satisfies JointBinding);
  if (!previous) rebaseAngle(object, binding);
  assertReachableTarget(object, binding);
  const drives = driveStates(object);
  if (
    binding.settingsVersion === object.settingsVersion &&
    sameDrives(binding.driveStates, drives)
  )
    return binding;
  if (!object.enabled) {
    if (binding.joint) backend.removeImpulseJoint(binding.joint, true);
    binding.joint = undefined;
  } else if (!binding.joint)
    binding.joint = createJoint(api, backend, object, binding);
  else applySettings(api, backend, object, binding.joint);
  binding.settingsVersion = object.settingsVersion;
  binding.driveStates = drives;
  return binding;
}

/** Rapier's shortest-arc motor cannot aim half a turn or more away from the current angle. */
function assertReachableTarget(object: Joint, binding: JointBinding): void {
  if (!(object instanceof RevoluteJoint) || !object.enabled) return;
  const drive = object.drive;
  if (!drive?.target || (drive.options.stiffness ?? 0) <= 0) return;
  if (
    Math.abs(drive.target.position - readBinding(object, binding).angle) >=
    Math.PI
  )
    throw new Error(
      "Revolute drive position must remain within pi radians of the current continuous angle; use intermediate targets for longer moves",
    );
}

function createJoint(
  api: typeof Rapier,
  backend: Rapier.World,
  object: Joint,
  binding: JointBinding,
): Rapier.ImpulseJoint {
  const [frame0, frame1] = binding.frames;
  const a = new Vector3().setFromMatrixPosition(frame0);
  const b = new Vector3().setFromMatrixPosition(frame1);
  const rotation0 = new Quaternion().setFromRotationMatrix(frame0);
  const rotation1 = new Quaternion().setFromRotationMatrix(frame1);
  const data = jointData(api, object, a, rotation0, b, rotation1);
  const joint = backend.createImpulseJoint(data, ...binding.bodies, true);
  try {
    const raw = backend.impulseJoints.raw;
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
    applySettings(api, backend, object, joint);
    return joint;
  } catch (error) {
    backend.removeImpulseJoint(joint, true);
    throw error;
  }
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

/** The mutable part of a joint: connected-body contacts and drives. */
function applySettings(
  api: typeof Rapier,
  backend: Rapier.World,
  object: Joint,
  joint: Rapier.ImpulseJoint,
): void {
  joint.setContactsEnabled(object.collideConnected);
  configureDrives(api, backend, object, joint);
  joint.body1().wakeUp();
  joint.body2().wakeUp();
}
