import { Mesh, type Object3D } from "three";
import { RigidBody } from "../body.js";
import { Trigger } from "../trigger.js";
import { Collider } from "./collider.js";

/**
 * What an owner's colliders are made from: its explicit colliders, or else a body's meshes.
 * A body's triggers are separate owners; a trigger contains neither triggers nor bodies.
 */
export function colliderSources(
  owner: RigidBody | Trigger,
): (Collider | Mesh)[] {
  const explicit: Collider[] = [];
  const meshes: Mesh[] = [];
  const collect = (object: Object3D): void => {
    if (object !== owner) {
      if (
        owner instanceof Trigger &&
        (object instanceof Trigger || object instanceof RigidBody)
      )
        throw new Error("Triggers cannot contain triggers or rigid bodies");
      if (object instanceof Trigger) return;
      if (object instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
    }
    if (object instanceof Collider) explicit.push(object);
    if (object instanceof Mesh) meshes.push(object);
    for (const child of object.children) collect(child);
  };
  collect(owner);
  if (explicit.length || owner instanceof Trigger) return explicit;
  return owner.options.colliders === false ? [] : meshes;
}
