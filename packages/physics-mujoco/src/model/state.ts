import type { MainModule } from "@mujoco/mujoco";
import {
  SphericalJoint,
  splitTransform,
  type Joint,
  type PhysicsVelocity,
  type RigidBody,
} from "@drawcall/physics";
import {
  dofState,
  initialVelocity,
  type JointBinding,
  type Motion,
} from "@drawcall/physics/backend";
import { Quaternion } from "three";
import type { Simulation } from "./compile.js";
import { name } from "./markup.js";
import { array, at, vector } from "../heap.js";
import { bodyId, pointVelocity, velocity, writeVelocity } from "../body.js";

/**
 * Carries motion and kinematic targets from the model a rebuild replaces into `next`. Joints
 * both models share keep their velocities exactly; new ones start from the motion of their
 * bodies, which bodies new to the simulation take from their initial velocity.
 */
export function carryState(
  api: MainModule,
  previous: Simulation | undefined,
  next: Simulation,
  bindings: ReadonlyMap<Joint, JointBinding>,
): void {
  const carried = previous
    ? copyShared(api, previous, next)
    : new Set<number>();
  const motion = carriedMotion(api, previous, next);
  const qvel = array(next.data.qvel);
  for (const [body, id] of next.bodies) {
    const joint = at(next.model.body_jntadr, id);
    if (
      joint >= 0 &&
      !carried.has(joint) &&
      at(next.model.jnt_type, joint) === api.mjtJoint.mjJNT_FREE.value
    )
      writeVelocity(api, next, id, carriedVelocity(api, previous, body));
  }
  for (const [j, coordinate] of next.coordinates) {
    if (carried.has(j)) continue;
    const axis =
      coordinate.kind === "axis" ? coordinate.joint.dof : coordinate.axis;
    const binding = bindings.get(coordinate.joint);
    if (!binding) throw new Error("Missing MuJoCo joint binding");
    qvel[coordinate.dof] = dofState(binding.read(motion), axis).velocity;
  }
  for (const joint of bindings.keys()) {
    if (!(joint instanceof SphericalJoint) || !joint.enabled) continue;
    const j = api.mj_name2id(
      next.model,
      api.mjtObj.mjOBJ_JOINT.value,
      name(joint),
    );
    if (j < 0) throw new Error(`Missing MuJoCo ball joint: ${name(joint)}`);
    if (carried.has(j)) continue;
    const { body0, body1 } = joint.options;
    const angular = motion.angular(body1);
    if (body0) angular.sub(motion.angular(body0));
    angular.applyQuaternion(
      new Quaternion()
        .setFromRotationMatrix(splitTransform(body1.matrixWorld).pose)
        .invert(),
    );
    qvel.set(angular.toArray(), at(next.model.jnt_dofadr, j));
  }
}

/**
 * Copies the velocities of the joints both models share on bodies `previous` still holds,
 * matched by name, and the kinematic targets; returns the joints it covered.
 */
function copyShared(
  api: MainModule,
  previous: Simulation,
  next: Simulation,
): Set<number> {
  const carried = new Set<number>();
  // A member that left no longer appears in `previous.bodies`, though its joints remain.
  const held = new Set(previous.bodies.values());
  for (let j = 0; j < next.model.njnt; j++) {
    const key = api.mj_id2name(next.model, api.mjtObj.mjOBJ_JOINT.value, j);
    const from = api.mj_name2id(
      previous.model,
      api.mjtObj.mjOBJ_JOINT.value,
      key,
    );
    if (from < 0 || !held.has(at(previous.model.jnt_bodyid, from))) continue;
    const start = at(previous.model.jnt_dofadr, from);
    const address = at(next.model.jnt_dofadr, j);
    // MuJoCo lays degrees of freedom out in joint order.
    const end =
      j + 1 < next.model.njnt
        ? at(next.model.jnt_dofadr, j + 1)
        : next.model.nv;
    array(next.data.qvel).set(
      array(previous.data.qvel).slice(start, start + end - address),
      address,
    );
    carried.add(j);
  }
  for (const [body, id] of next.targets) {
    const from = previous.targets.get(body);
    if (from === undefined) continue;
    const source = at(previous.model.body_mocapid, from);
    const target = at(next.model.body_mocapid, id);
    array(next.data.mocap_pos).set(
      array(previous.data.mocap_pos).slice(source * 3, source * 3 + 3),
      target * 3,
    );
    array(next.data.mocap_quat).set(
      array(previous.data.mocap_quat).slice(source * 4, source * 4 + 4),
      target * 4,
    );
  }
  return carried;
}

/** Simulated motion for bodies the previous model held; initial motion for the rest. */
function carriedMotion(
  api: MainModule,
  previous: Simulation | undefined,
  next: Simulation,
): Motion {
  return {
    angular: (body) => carriedVelocity(api, previous, body).angular,
    velocityAt: (body, point) => {
      const id = previous?.bodies.get(body);
      if (previous && id !== undefined)
        return pointVelocity(api, previous, id, point);
      const { linear, angular } = initialVelocity(body);
      const center = vector(next.model.body_ipos, bodyId(next, body) * 3);
      center.applyMatrix4(splitTransform(body.matrixWorld).pose);
      return angular.cross(point.clone().sub(center)).add(linear);
    },
  };
}

/** The simulated velocity of a body the previous model held; the initial one of the rest. */
function carriedVelocity(
  api: MainModule,
  previous: Simulation | undefined,
  body: RigidBody,
): PhysicsVelocity {
  const id = previous?.bodies.get(body);
  if (previous && id !== undefined) return velocity(api, previous, id);
  return initialVelocity(body);
}
