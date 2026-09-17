import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  AxisJoint,
  DistanceJoint,
  FixedJoint,
  GenericJoint,
  PrismaticJoint,
  RevoluteJoint,
  ScalarJoint,
  SphericalJoint,
  authoredJointReading,
  jointDofs,
  type Joint,
  type JointDof,
  type JointDrive,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";

type API = typeof Rapier;

/** Rapier types per-axis motors only on spherical joints; the raw set takes plain axis and model indices. */
function rapierAxis(api: API, axis: JointDof): number {
  const { LinX, LinY, LinZ, AngX, AngY, AngZ } = api.JointAxis;
  return {
    transX: LinX,
    transY: LinY,
    transZ: LinZ,
    rotX: AngX,
    rotY: AngY,
    rotZ: AngZ,
  }[axis];
}
function rapierMask(api: API, axis: JointDof): number {
  const { LinX, LinY, LinZ, AngX, AngY, AngZ } = api.JointAxesMask;
  return {
    transX: LinX,
    transY: LinY,
    transZ: LinZ,
    rotX: AngX,
    rotY: AngY,
    rotZ: AngZ,
  }[axis];
}

/** Every drive slot of a joint with its current drive: one for scalar joints, six for generic joints. */
function slots(object: Joint): [JointDof, JointDrive | undefined][] {
  if (object instanceof ScalarJoint)
    return [
      [object instanceof AxisJoint ? object.dof : "transX", object.drive],
    ];
  if (object instanceof GenericJoint)
    return jointDofs.map((axis) => [axis, object.getDrive(axis)]);
  return [];
}
type DriveState = [JointDrive | undefined, number];
/** Which drive fills each slot and how it was last configured, for change detection. */
function driveStates(object: Joint): DriveState[] {
  return slots(object).map(([, drive]) => [
    drive,
    drive?.settingsVersion ?? -1,
  ]);
}
function sameDrives(a: DriveState[], b: DriveState[]): boolean {
  return (
    a.length === b.length &&
    a.every(
      ([drive, version], i) => b[i]?.[0] === drive && b[i]?.[1] === version,
    )
  );
}

function createJoint(
  api: API,
  world: Rapier.World,
  object: Joint,
  frame0: Matrix4,
  frame1: Matrix4,
  body0: Rapier.RigidBody,
  body1: Rapier.RigidBody,
): Rapier.ImpulseJoint {
  const a = new Vector3().setFromMatrixPosition(frame0);
  const b = new Vector3().setFromMatrixPosition(frame1);
  const rotation0 = new Quaternion().setFromRotationMatrix(frame0);
  const rotation1 = new Quaternion().setFromRotationMatrix(frame1);
  const x = new Vector3(1, 0, 0);
  let data: Rapier.JointData;
  if (object instanceof FixedJoint)
    data = api.JointData.fixed(a, rotation0, b, rotation1);
  else if (object instanceof SphericalJoint)
    data = api.JointData.spherical(a, b);
  else if (object instanceof DistanceJoint) {
    if (object.limits[0] !== 0)
      throw new Error(
        "Rapier distance joints only support a zero minimum distance.",
      );
    // A spring joint is Rapier's driven distance constraint; its rope limit is set below.
    data = api.JointData.spring(0, 0, 0, a, b);
  } else if (object instanceof RevoluteJoint)
    data = api.JointData.revolute(a, b, x);
  else if (object instanceof PrismaticJoint)
    data = api.JointData.prismatic(a, b, x);
  else if (object instanceof GenericJoint) {
    let locked = 0;
    for (const axis of jointDofs)
      if (object.dofs[axis] === "locked") locked |= rapierMask(api, axis);
    data = api.JointData.generic(a, b, x, locked);
  } else throw new Error(`Unsupported Rapier joint: ${object.type}`);
  const result = world.createImpulseJoint(data, body0, body1, true);
  try {
    const raw = world.impulseJoints.raw;
    if (object instanceof DistanceJoint && Number.isFinite(object.limits[1]))
      raw.jointSetLimits(
        result.handle,
        rapierAxis(api, "transX"),
        0,
        object.limits[1],
      );
    if (object instanceof AxisJoint) {
      const axis =
        object.options.axis === "X"
          ? new Vector3(1, 0, 0)
          : (object.options.axis ?? "Y") === "Y"
            ? new Vector3(0, 1, 0)
            : new Vector3(0, 0, 1);
      const rotation = new Quaternion().setFromUnitVectors(x, axis);
      result.setLocalFrame1(a, rotation0.clone().multiply(rotation));
      result.setLocalFrame2(b, rotation1.clone().multiply(rotation));
      if (!(result instanceof api.UnitImpulseJoint))
        throw new Error("Expected a Rapier unit joint");
      if (object.options.limits) result.setLimits(...object.options.limits);
    }
    if (object instanceof GenericJoint) {
      result.setLocalFrame1(a, rotation0);
      result.setLocalFrame2(b, rotation1);
      for (const axis of jointDofs) {
        const motion = object.dofs[axis];
        if (typeof motion !== "string")
          raw.jointSetLimits(
            result.handle,
            rapierAxis(api, axis),
            motion[0],
            motion[1],
          );
      }
    }
    configure(api, world, object, result);
    return result;
  } catch (error) {
    world.removeImpulseJoint(result, true);
    throw error;
  }
}

function configure(
  api: API,
  world: Rapier.World,
  object: Joint,
  impulse: Rapier.ImpulseJoint,
): void {
  impulse.setContactsEnabled(object.collideConnected);
  for (const [axis, drive] of slots(object)) {
    const goal = drive?.target;
    const gains =
      (drive?.options.stiffness ?? 0) > 0 || (drive?.options.damping ?? 0) > 0;
    // A Rapier motor without gains freezes the body; effort-only drives act through per-step forces instead.
    const native =
      drive && goal && gains ? { options: drive.options, goal } : undefined;
    const model: number =
      native?.options.model === "acceleration"
        ? api.MotorModel.AccelerationBased
        : api.MotorModel.ForceBased;
    const maxForce = native ? (native.options.maxForce ?? Number.MAX_VALUE) : 0;
    const position = native?.goal.position ?? 0;
    const settings = [
      // Rapier chooses the shortest arc and only wraps its error once.
      axis.startsWith("rot")
        ? Math.atan2(Math.sin(position), Math.cos(position))
        : position,
      native?.goal.velocity ?? 0,
      native?.options.stiffness ?? 0,
      native?.options.damping ?? 0,
    ] as const;
    if (impulse instanceof api.UnitImpulseJoint) {
      impulse.configureMotorModel(model);
      impulse.setMotorMaxForce(maxForce);
      impulse.configureMotor(...settings);
    } else {
      const raw = world.impulseJoints.raw;
      const index = rapierAxis(api, axis);
      raw.jointConfigureMotorModel(impulse.handle, index, model);
      raw.jointSetMotorMaxForce(impulse.handle, index, maxForce);
      raw.jointConfigureMotor(impulse.handle, index, ...settings);
    }
  }
  impulse.body1().wakeUp();
  impulse.body2().wakeUp();
}

/** Applies each drive's effort term as a force pair for one step, capped by its `maxForce`. */
export function applyEfforts(
  object: Joint,
  impulse: Rapier.ImpulseJoint,
): void {
  // Rapier numbers the two bodies from one.
  const body0 = impulse.body1();
  const body1 = impulse.body2();
  const anchor = (body: Rapier.RigidBody, point: Rapier.Vector) =>
    new Vector3()
      .copy(point)
      .applyQuaternion(new Quaternion().copy(body.rotation()))
      .add(body.translation());
  for (const [axis, drive] of slots(object)) {
    const effort = drive?.target?.effort ?? 0;
    if (!drive || effort === 0) continue;
    const cap = drive.options.maxForce ?? Infinity;
    const value = Math.max(-cap, Math.min(cap, effort));
    const anchor0 = anchor(body0, impulse.anchor1());
    const anchor1 = anchor(body1, impulse.anchor2());
    let direction: Vector3;
    if (object instanceof DistanceJoint) {
      direction = anchor1.clone().sub(anchor0);
      if (direction.lengthSq() < 1e-24) continue;
      direction.normalize();
    } else {
      const index = jointDofs.indexOf(axis) % 3;
      direction = new Vector3()
        .setComponent(index, 1)
        .applyQuaternion(new Quaternion().copy(impulse.frameX1()))
        .applyQuaternion(new Quaternion().copy(body0.rotation()));
    }
    const load = direction.multiplyScalar(value);
    if (axis.startsWith("rot")) {
      body1.addTorque(load, true);
      body0.addTorque(load.clone().negate(), true);
    } else {
      body1.addForceAtPoint(load, anchor1, true);
      body0.addForceAtPoint(load.clone().negate(), anchor0, true);
    }
  }
}

export interface JointBinding {
  target: Rapier.ImpulseJoint | undefined;
  frames: readonly [Matrix4, Matrix4];
  bodies: readonly [Rapier.RigidBody, Rapier.RigidBody];
  settingsVersion: number;
  driveStates: DriveState[];
  angle?: number;
  position?: number;
}

export function prepareJoint(
  api: API,
  world: Rapier.World,
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
      target: undefined,
      settingsVersion: -1,
      driveStates: [],
    } satisfies JointBinding);
  if (!previous) sampleJoint(object, binding, true);
  const goal =
    object instanceof RevoluteJoint ? object.drive?.target : undefined;
  if (
    object.enabled &&
    object instanceof RevoluteJoint &&
    goal &&
    (object.drive?.options.stiffness ?? 0) > 0 &&
    Math.abs(goal.position - readBinding(object, binding).angle) >= Math.PI
  )
    throw new Error(
      "Revolute drive position must remain within pi radians of the current continuous angle; use intermediate targets for longer moves",
    );
  const drives = driveStates(object);
  if (
    binding.settingsVersion === object.settingsVersion &&
    sameDrives(binding.driveStates, drives)
  )
    return binding;
  if (!object.enabled) {
    if (binding.target) world.removeImpulseJoint(binding.target, true);
    binding.target = undefined;
  } else if (!binding.target) {
    binding.target = createJoint(
      api,
      world,
      object,
      binding.frames[0],
      binding.frames[1],
      body0,
      body1,
    );
  } else configure(api, world, object, binding.target);
  binding.settingsVersion = object.settingsVersion;
  binding.driveStates = drives;
  return binding;
}

function measureJoint(object: Joint, binding: JointBinding) {
  return authoredJointReading(object, binding.frames, (body, point) => {
    const target = binding.bodies[body === object.options.body0 ? 0 : 1];
    return new Vector3().copy(target.velocityAtPoint(point));
  });
}

export function readBinding(object: Joint, binding: JointBinding) {
  const state = measureJoint(object, binding);
  if (object instanceof RevoluteJoint && binding.position !== undefined)
    return { ...state, angle: binding.position };
  return state;
}

export function sampleJoint(
  object: Joint,
  binding: JointBinding,
  rebase = false,
): void {
  if (!(object instanceof RevoluteJoint)) return;
  const angle = measureJoint(object, binding).angle;
  if (rebase || binding.angle === undefined || binding.position === undefined)
    binding.position = angle;
  else
    binding.position += Math.atan2(
      Math.sin(angle - binding.angle),
      Math.cos(angle - binding.angle),
    );
  binding.angle = angle;
}
