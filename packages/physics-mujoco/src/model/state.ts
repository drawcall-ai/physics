import type { MainModule } from "@mujoco/mujoco";
import {
  SphericalJoint,
  splitTransform,
  type Joint,
  type RigidBody,
} from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import type { Compiled } from "./compile.js";
import { array, at, name } from "../values.js";
import { writeVelocity } from "../motion.js";

/** Preserve motion and commanded poses when topology changes rebuild the model. */
export function captureState(
  api: MainModule,
  previous: Compiled | undefined,
  bodies: Iterable<RigidBody>,
  joints: Iterable<Joint>,
): (next: Compiled) => void {
  const velocities = new Map(
    [...bodies].map((body) => [body, body.getVelocity()]),
  );
  const targets = new Map<
    RigidBody,
    { position: number[]; rotation: number[] }
  >();
  if (previous) {
    for (const [body, id] of previous.targets) {
      const mocap = at(previous.model.body_mocapid, id);
      targets.set(body, {
        position: Array.from(
          array(previous.data.mocap_pos).slice(mocap * 3, mocap * 3 + 3),
        ),
        rotation: Array.from(
          array(previous.data.mocap_quat).slice(mocap * 4, mocap * 4 + 4),
        ),
      });
    }
  }
  const speeds = new Map<string, number[]>();
  if (previous)
    for (let j = 0; j < previous.model.njnt; j++) {
      const key = api.mj_id2name(
        previous.model,
        api.mjtObj.mjOBJ_JOINT.value,
        j,
      );
      const start = at(previous.model.jnt_dofadr, j),
        type = at(previous.model.jnt_type, j);
      speeds.set(
        key,
        Array.from(
          array(previous.data.qvel).slice(
            start,
            start +
              (type === api.mjtJoint.mjJNT_FREE.value
                ? 6
                : type === api.mjtJoint.mjJNT_BALL.value
                  ? 3
                  : 1),
          ),
        ),
      );
    }
  return (next) => {
    for (const [body, id] of next.targets) {
      const pose = targets.get(body);
      if (!pose) continue;
      const mocap = at(next.model.body_mocapid, id);
      array(next.data.mocap_pos).set(pose.position, mocap * 3);
      array(next.data.mocap_quat).set(pose.rotation, mocap * 4);
    }
    for (const [body, id] of next.bodies) {
      const value = velocities.get(body);
      if (body.bodyType !== "static" && next.roots.has(body) && value)
        writeVelocity(api, next, id, value);
    }
    for (let j = 0; j < next.model.njnt; j++) {
      if (at(next.model.jnt_type, j) === api.mjtJoint.mjJNT_FREE.value)
        continue;
      const key = api.mj_id2name(next.model, api.mjtObj.mjOBJ_JOINT.value, j);
      const values = speeds.get(key);
      const address = at(next.model.jnt_dofadr, j);
      if (values) {
        array(next.data.qvel).set(values, address);
        continue;
      }
      const coordinate = next.coordinates.get(j);
      if (coordinate?.kind === "axis") {
        array(next.data.qvel)[address] = coordinate.joint.getState().velocity;
      } else if (coordinate?.kind === "generic") {
        array(next.data.qvel)[address] = coordinate.joint.getState(
          coordinate.axis,
        ).velocity;
      } else {
        const joint = [...joints].find((joint) => name(joint) === key);
        if (!(joint instanceof SphericalJoint))
          throw new Error(`Missing spherical joint binding: ${key}`);
        const { body0, body1 } = joint.options;
        const angular = body1
          .getVelocity()
          .angular.sub(body0?.getVelocity().angular ?? new Vector3());
        angular.applyQuaternion(
          new Quaternion()
            .setFromRotationMatrix(splitTransform(body1.matrixWorld).pose)
            .invert(),
        );
        array(next.data.qvel).set(angular.toArray(), address);
      }
    }
  };
}
