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

/** Save native joint speeds by name while body COM velocities cover newly free roots. */
export function captureVelocities(
  api: MainModule,
  previous: Compiled | undefined,
  bodies: Iterable<RigidBody>,
  joints: Iterable<Joint>,
): (next: Compiled) => void {
  const velocities = new Map(
    [...bodies].map((body) => [body, body.getVelocity()]),
  );
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
            start + (type === 0 ? 6 : type === 1 ? 3 : 1),
          ),
        ),
      );
    }
  return (next) => {
    for (const [body, id] of next.bodies) {
      const value = velocities.get(body);
      if (body.bodyType === "dynamic" && next.roots.has(body) && value)
        writeVelocity(next, id, value);
    }
    for (let j = 0; j < next.model.njnt; j++) {
      if (at(next.model.jnt_type, j) === 0) continue;
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
