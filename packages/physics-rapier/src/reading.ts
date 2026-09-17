import {
  RevoluteJoint,
  type Joint,
  type JointReading,
  authoredJointReading,
  wrapAngle,
} from "@drawcall/physics";
import { Vector3 } from "three";
import type { JointBinding } from "./joints.js";

function measure(object: Joint, binding: JointBinding): JointReading {
  return authoredJointReading(object, binding.frames, (body, point) => {
    const target = binding.bodies[body === object.options.body0 ? 0 : 1];
    return new Vector3().copy(target.velocityAtPoint(point));
  });
}

/** The joint's reading, with the revolute angle continued across turns. */
export function readBinding(
  object: Joint,
  binding: JointBinding,
): JointReading {
  const reading = measure(object, binding);
  if (object instanceof RevoluteJoint && binding.angle)
    return { ...reading, angle: binding.angle.continuous };
  return reading;
}

/** Restarts turn counting from the current angle, as creation, teleport, and reset do. */
export function rebaseAngle(object: Joint, binding: JointBinding): void {
  if (!(object instanceof RevoluteJoint)) return;
  const sampled = measure(object, binding).angle;
  binding.angle = { sampled, continuous: sampled };
}

/** Adds the wrapped change since the last sample; motion must stay below π per step. */
export function trackAngle(object: Joint, binding: JointBinding): void {
  if (!(object instanceof RevoluteJoint)) return;
  const sampled = measure(object, binding).angle;
  const previous = binding.angle ?? { sampled, continuous: sampled };
  binding.angle = {
    sampled,
    continuous: previous.continuous + wrapAngle(sampled - previous.sampled),
  };
}
