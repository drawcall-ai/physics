import type { Matrix4, Object3D } from "three";
import type { RigidBody } from "../body.js";
import { setWorldPose, splitTransform } from "../transforms.js";
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

/**
 * Moves `body` rigidly to `pose` in the scene, with the dynamic bodies `joints` hold to it.
 * Returns the moved bodies and the actions that put their transforms back.
 */
export function placeAssembly(
  body: RigidBody,
  pose: Matrix4,
  joints: Iterable<Joint>,
): { moved: Set<RigidBody>; restores: (() => void)[] } {
  body.validate();
  const delta = pose
    .clone()
    .multiply(splitTransform(body.matrixWorld).pose.invert());
  const moved = assembly(body, joints);
  const poses = [...moved].map((member) => {
    if (member === body) return [member, pose] as const;
    member.validate();
    return [
      member,
      delta.clone().multiply(splitTransform(member.matrixWorld).pose),
    ] as const;
  });
  const restores = [...moved].map(saveTransform);
  for (const [member, memberPose] of poses) setWorldPose(member, memberPose);
  return { moved, restores };
}

/** The joints in the whole hierarchy that holds `body`. */
export function hierarchyJoints(body: RigidBody): Joint[] {
  const joints: Joint[] = [];
  top(body).traverse((node) => {
    if (node instanceof Joint) joints.push(node);
  });
  return joints;
}

function top(object: Object3D): Object3D {
  return object.parent ? top(object.parent) : object;
}

/** Returns a function that puts the object's local transform back as it is now. */
function saveTransform(object: Object3D): () => void {
  const position = object.position.clone();
  const quaternion = object.quaternion.clone();
  const scale = object.scale.clone();
  return () => {
    object.position.copy(position);
    object.quaternion.copy(quaternion);
    object.scale.copy(scale);
    object.updateMatrix();
    object.updateMatrixWorld(true);
  };
}
