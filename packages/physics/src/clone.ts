import { Object3D } from "three";
import { Joint } from "./joints/joint.js";

/** Copy bodies before constructing joints so immutable connections point at their copies. */
export function clone<T extends Object3D>(root: T): T {
  const copies = new Map<Object3D, Object3D>();
  const joints: Joint[] = [];
  function copy<U extends Object3D>(source: U): U {
    const target = source.clone(false);
    copies.set(source, target);
    return target;
  }
  let result = root instanceof Joint ? undefined : copy(root);
  root.traverse((source) => {
    if (source === root) return;
    if (source instanceof Joint) joints.push(source);
    else copy(source);
  });
  for (const joint of joints) {
    const target = joint.cloneWithBodies(copies, false);
    copies.set(joint, target);
  }
  if (root instanceof Joint) {
    const target = root.cloneWithBodies(copies, false);
    copies.set(root, target);
    result = target;
  }
  for (const [source, target] of copies) {
    for (const child of source.children) {
      const copiedChild = copies.get(child);
      if (!copiedChild) throw new Error("Missing cloned child");
      target.add(copiedChild);
    }
  }
  if (!result) throw new Error("Missing cloned root");
  return result;
}
