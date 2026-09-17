import { Group, Matrix4, Object3D, Scene } from "three";
import { Collider, Joint, RigidBody, splitTransform } from "@drawcall/physics";
import { PhysicsUSDScene } from "../scene.js";
import { Prim } from "./prim.js";

/**
 * Mirrors the scene as prims named P0, P1, … and as a visual clone with the same names, so
 * Three's exporter and the physics layer describe identical paths. Colliders and joints are
 * left out; bodies flatten to their rigid pose and hand their scale to their children.
 */
export class Hierarchy {
  readonly paths = new Map<Object3D, string>();
  private readonly prims = new Map<Object3D, Prim>();
  private count = 0;

  mirror(
    object: Object3D,
    parent: Prim,
    parentPath: string,
  ): Object3D | undefined {
    if (object instanceof Collider || object instanceof Joint) return undefined;
    const name = `P${this.count++}`;
    const path = `${parentPath}/${name}`;
    const clone =
      object instanceof RigidBody ||
      object instanceof Scene ||
      object instanceof PhysicsUSDScene
        ? new Group().copy(object, false)
        : object.clone(false);
    clone.name = name;
    clone.matrixAutoUpdate = false;
    clone.matrix.copy(exportMatrix(object));
    const prim = new Prim(name, "");
    prim.displayName = object.name;
    if (!object.visible) prim.properties.push('token visibility = "invisible"');
    parent.children.push(prim);
    this.paths.set(object, path);
    this.prims.set(object, prim);
    for (const child of object.children) {
      const result = this.mirror(child, prim, path);
      if (result) clone.add(result);
    }
    return clone;
  }

  prim(object: Object3D): Prim {
    const prim = this.prims.get(object);
    if (!prim) throw new Error("Body missing from USD hierarchy");
    return prim;
  }
}

function exportMatrix(object: Object3D): Matrix4 {
  const matrix = object.matrix.clone();
  if (object instanceof RigidBody) {
    matrix.copy(splitTransform(object.matrixWorld).pose);
    if (object.parent)
      matrix.premultiply(object.parent.matrixWorld.clone().invert());
  }
  if (object.parent instanceof RigidBody)
    matrix.premultiply(
      new Matrix4().makeScale(
        ...splitTransform(object.parent.matrixWorld).scale.toArray(),
      ),
    );
  return matrix;
}
