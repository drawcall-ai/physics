import { Object3D } from "three";
import { RigidBody } from "./body.js";
import { Joint } from "./joints.js";

/** Clone a hierarchy and reconnect joints to bodies copied within it. */
export function clone<T extends Object3D>(root: T): T {
  const copies = new Map<Object3D, Object3D>();
  const created: (RigidBody | Joint)[] = [];
  function copy<U extends Object3D>(source: U): U {
    const target = source.clone(false);
    if (target instanceof RigidBody || target instanceof Joint)
      created.push(target);
    copies.set(source, target);
    for (const child of source.children) target.add(copy(child));
    // Finish native container ownership after its descendants exist.
    target.copy(source, false);
    return target;
  }
  try {
    const result = copy(root);
    for (const [source, target] of copies) {
      if (source instanceof Joint && target instanceof Joint)
        target.copy(source, false, copies);
    }
    return result;
  } catch (error) {
    for (const object of created.reverse()) object.dispose();
    throw error;
  }
}
