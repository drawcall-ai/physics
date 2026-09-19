import {
  AxisJoint,
  FixedJoint,
  GenericJoint,
  SphericalJoint,
  jointDofs,
  axisVector,
  type Joint,
  type JointDrive,
  type JointReading,
} from "@drawcall/physics";
import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import { name } from "../values.js";

export interface JointRecord {
  frames: [Matrix4, Matrix4];
  angle: number;
  sampled: number;
}
export type Coordinate = {
  name: string;
  position: number;
} & (
  | { kind: "axis"; joint: AxisJoint }
  | { kind: "generic"; joint: GenericJoint; axis: (typeof jointDofs)[number] }
);
export function jointXml(
  joint: Joint,
  record: JointRecord,
  reading: JointReading,
  coordinates: Coordinate[],
): string {
  validateAlignment(joint, reading);
  const frame = record.frames[1];
  const position = new Vector3()
    .setFromMatrixPosition(frame)
    .toArray()
    .join(" ");
  const rotation = new Quaternion().setFromRotationMatrix(frame);
  const scalar = (
    type: "slide" | "hinge",
    axis: Vector3,
    limits: readonly [number, number] | undefined,
    coordinate: Coordinate,
  ) => {
    coordinates.push(coordinate);
    const key = coordinate.name;
    // Back-EMF resists motion whenever the motor is connected, so it belongs on the joint, where
    // MuJoCo integrates it implicitly. It is what settles a saturated drive at its rated speed.
    const backEmf = driveOf(coordinate)?.backEmf;
    const damping = backEmf === undefined ? "" : ` damping="${backEmf}"`;
    if (limits && limits[0] === limits[1])
      throw new Error(
        "MuJoCo scalar limits must have a nonzero range; use a locked dof instead",
      );
    return `<joint name="${key}" type="${type}" pos="${position}" axis="${axis.applyQuaternion(rotation).toArray().join(" ")}" ${limits ? `range="${limits.join(" ")}" limited="true"` : 'limited="false"'}${damping}/>`;
  };
  if (joint instanceof FixedJoint) return "";
  if (joint instanceof SphericalJoint)
    return `<joint name="${name(joint)}" type="ball" pos="${position}"/>`;
  if (joint instanceof AxisJoint)
    return scalar(
      joint.dof === "rotX" ? "hinge" : "slide",
      axisVector(joint.options.axis),
      joint.limits,
      {
        kind: "axis",
        joint,
        name: name(joint),
        position: joint.dof === "rotX" ? reading.angle : reading.translation.x,
      },
    );
  if (joint instanceof GenericJoint)
    return jointDofs
      .map((axis, i) => {
        const motion = joint.dofs[axis];
        if (motion === "locked") return "";
        const direction = new Vector3().setComponent(i % 3, 1);
        const state = joint.getState(axis);
        return scalar(
          i < 3 ? "slide" : "hinge",
          direction,
          motion === "free" ? undefined : motion,
          {
            kind: "generic",
            joint,
            axis,
            name: name(joint) + axis,
            position: state.position,
          },
        );
      })
      .join("");
  throw new Error(`Unsupported MuJoCo joint: ${joint.constructor.name}`);
}
export function driveOf(coordinate: Coordinate): JointDrive | undefined {
  return coordinate.kind === "axis"
    ? coordinate.joint.drive
    : coordinate.joint.getDrive(coordinate.axis);
}

function validateAlignment(joint: Joint, reading: JointReading): void {
  const { translation, rotation } = reading;
  const angles = new Euler().setFromQuaternion(rotation, "XYZ");
  const values = [...translation.toArray(), angles.x, angles.y, angles.z];
  const locked =
    joint instanceof GenericJoint
      ? jointDofs.map((axis) => joint.dofs[axis] === "locked")
      : joint instanceof SphericalJoint
        ? [true, true, true, false, false, false]
        : joint instanceof AxisJoint
          ? joint.dof === "rotX"
            ? [true, true, true, false, true, true]
            : [false, true, true, true, true, true]
          : [true, true, true, true, true, true];
  if (values.some((v, i) => locked[i] && Math.abs(v) > 1e-5))
    throw new Error(
      "MuJoCo tree joints require initially aligned locked coordinates; align the body poses with their joint frames",
    );
}
