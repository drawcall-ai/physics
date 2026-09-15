import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  AxisJoint,
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
    if (object.limits[0] !== 0)
      throw new Error(
        "Rapier distance joints only support a zero minimum distance.",
      );
    data = api.JointData.rope(object.limits[1], a, b);
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
    result.setContactsEnabled(object.collideConnected);
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
      if (object.limits) result.setLimits(...object.limits);
      configure(api, object, result);
    }
    return result;
  } catch (error) {
    world.removeImpulseJoint(result, true);
    throw error;
  }
}

export function configure(
  api: API,
  object: Joint,
  target: Rapier.ImpulseJoint,
): void {
  target.setContactsEnabled(object.collideConnected);
  if (!(object instanceof RevoluteJoint || object instanceof PrismaticJoint))
    return;
  if (!(target instanceof api.UnitImpulseJoint))
    throw new Error("Expected a Rapier unit joint.");
  target.setLimits(...(object.limits ?? [-Number.MAX_VALUE, Number.MAX_VALUE]));
}

export function applyEffort(
  object: AxisJoint,
  target: Rapier.ImpulseJoint,
  value: number,
): void {
  const first = target.body1();
  const second = target.body2();
  const axis = new Vector3(1, 0, 0)
    .applyQuaternion(new Quaternion().copy(target.frameX1()))
    .applyQuaternion(new Quaternion().copy(first.rotation()));
  const effort = axis.multiplyScalar(value);
  if (object instanceof RevoluteJoint) {
    second.addTorque(effort, true);
    first.addTorque(effort.clone().negate(), true);
    return;
  }
  const anchor = (body: Rapier.RigidBody, point: Rapier.Vector) =>
    new Vector3()
      .copy(point)
      .applyQuaternion(new Quaternion().copy(body.rotation()))
      .add(body.translation());
  second.addForceAtPoint(effort, anchor(second, target.anchor2()), true);
  first.addForceAtPoint(
    effort.clone().negate(),
    anchor(first, target.anchor1()),
    true,
  );
}
