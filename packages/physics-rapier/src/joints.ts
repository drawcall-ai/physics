import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  jointState,
  DistanceJoint,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  SphericalJoint,
  type Joint,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";

type API = typeof Rapier;

export function joint(
  api: API,
  world: Rapier.World,
  object: Joint,
  frame0: Matrix4,
  frame1: Matrix4,
  first: Rapier.RigidBody,
  second: Rapier.RigidBody,
): Rapier.ImpulseJoint {
  const a = new Vector3().setFromMatrixPosition(frame0);
  const b = new Vector3().setFromMatrixPosition(frame1);
  const rotation0 = new Quaternion().setFromRotationMatrix(frame0);
  const rotation1 = new Quaternion().setFromRotationMatrix(frame1);
  let data: Rapier.JointData;
  if (object instanceof FixedJoint)
    data = api.JointData.fixed(a, rotation0, b, rotation1);
  else if (object instanceof SphericalJoint)
    data = api.JointData.spherical(a, b);
  else if (object instanceof DistanceJoint) {
    if (object.options.limits[0] !== 0)
      throw new Error(
        "Rapier distance joints only support a zero minimum distance.",
      );
    data = api.JointData.rope(object.options.limits[1], a, b);
  } else if (
    object instanceof RevoluteJoint ||
    object instanceof PrismaticJoint
  ) {
    const axis = new Vector3(1, 0, 0);
    data =
      object instanceof RevoluteJoint
        ? api.JointData.revolute(a, b, axis)
        : api.JointData.prismatic(a, b, axis);
  } else throw new Error(`Unsupported Rapier joint: ${object.type}`);
  const result = world.createImpulseJoint(data, first, second, true);
  try {
    result.setContactsEnabled(object.options.collideConnected ?? false);
    if (object instanceof RevoluteJoint || object instanceof PrismaticJoint) {
      const axis =
        object.options.axis === "X"
          ? new Vector3(1, 0, 0)
          : (object.options.axis ?? "Y") === "Y"
            ? new Vector3(0, 1, 0)
            : new Vector3(0, 0, 1);
      const rotation = new Quaternion().setFromUnitVectors(
        new Vector3(1, 0, 0),
        axis,
      );
      result.setLocalFrame1(a, rotation0.clone().multiply(rotation));
      result.setLocalFrame2(b, rotation1.clone().multiply(rotation));
      if (!(result instanceof api.UnitImpulseJoint))
        throw new Error("Rapier returned an unexpected joint type.");
      if (object.options.limits) result.setLimits(...object.options.limits);
      drive(api, object, result);
    }
    return result;
  } catch (error) {
    world.removeImpulseJoint(result, true);
    throw error;
  }
}

export function drive(
  api: API,
  object: Joint,
  target: Rapier.ImpulseJoint,
): void {
  target.setContactsEnabled(object.options.collideConnected ?? false);
  if (!(object instanceof RevoluteJoint || object instanceof PrismaticJoint))
    return;
  if (!(target instanceof api.UnitImpulseJoint))
    throw new Error("Expected a Rapier unit joint.");
  target.setLimits(
    ...(object.options.limits ?? [-Number.MAX_VALUE, Number.MAX_VALUE]),
  );
  const value = object.options.drive;
  target.configureMotorModel(
    value?.type === "acceleration"
      ? api.MotorModel.AccelerationBased
      : api.MotorModel.ForceBased,
  );
  target.setMotorMaxForce(value?.maxForce ?? Number.MAX_VALUE);
  target.configureMotor(
    value?.targetPosition ?? 0,
    value?.targetVelocity ?? 0,
    value?.stiffness ?? 0,
    value?.damping ?? 0,
  );
}

export function readJointState(target: Rapier.ImpulseJoint) {
  const first = target.body1(),
    second = target.body2();
  const frame = (
    body: Rapier.RigidBody,
    position: Rapier.Vector,
    rotation: Rapier.Rotation,
  ): Matrix4 => {
    const matrix = new Matrix4().compose(
      new Vector3().copy(body.translation()),
      new Quaternion().copy(body.rotation()),
      new Vector3(1, 1, 1),
    );
    return matrix.multiply(
      new Matrix4().compose(
        new Vector3().copy(position),
        new Quaternion().copy(rotation),
        new Vector3(1, 1, 1),
      ),
    );
  };
  return jointState(
    frame(first, target.anchor1(), target.frameX1()),
    frame(second, target.anchor2(), target.frameX2()),
    new Vector3().copy(first.angvel()),
    new Vector3().copy(second.angvel()),
  );
}
