import type { Object3D } from "three";
import type { RigidBody } from "../body.js";
import { Joint } from "./joint.js";
import { DistanceJoint } from "./kinds.js";
import { GenericJoint, jointDofs } from "./generic.js";

/** A generic joint that leaves every degree of freedom free constrains nothing. */
export function unconstrained(joint: Joint): boolean {
  return (
    joint instanceof GenericJoint &&
    jointDofs.every((axis) => joint.dofs[axis] === "free")
  );
}

/** Joints that hold their bodies as one articulated assembly: enabled and neither a rope nor unconstrained. */
export function treeJoint(joint: Joint): boolean {
  return (
    joint.enabled && !(joint instanceof DistanceJoint) && !unconstrained(joint)
  );
}

/** The bodies that move as one with `body`: itself and every dynamic body reachable through tree joints. */
export function assembly(
  body: RigidBody,
  joints: Iterable<Joint>,
): Set<RigidBody> {
  const members = new Set([body]);
  const links = [...joints].filter(treeJoint);
  for (const member of members)
    for (const { options } of links) {
      const other =
        options.body0 === member
          ? options.body1
          : options.body1 === member
            ? options.body0
            : null;
      if (other?.bodyType === "dynamic") members.add(other);
    }
  return members;
}

/** The joints under its world's scene, or else in the whole hierarchy that holds it. */
export function hierarchyJoints(body: RigidBody): Joint[] {
  const joints: Joint[] = [];
  (body.world?.scene ?? top(body)).traverse((node) => {
    if (node instanceof Joint) joints.push(node);
  });
  return joints;
}

function top(object: Object3D): Object3D {
  return object.parent ? top(object.parent) : object;
}
