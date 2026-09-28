import {
  Collider,
  MeshCollider,
  RigidBody,
  colliderSources,
  splitTransform,
  geometryVersion,
  type Joint,
  type Trigger,
} from "@drawcall/physics";
import { Mesh, type Object3D, Vector3 } from "three";

/**
 * Detects authored changes without decomposing transforms: versions, each owner's world scale,
 * and its collider sources with the local transforms from them up to the owner, which physics
 * never writes. Moving bodies, and cameras or triggers moved inside them, read the same values.
 */
export class Changes {
  private scales = new Map<Object3D, Vector3>();
  private values: unknown[] = [];

  /** The new values and validated scales, or undefined when nothing changed since the commit. */
  changed(
    bodies: Iterable<RigidBody>,
    joints: Iterable<Joint>,
    triggers: Iterable<Trigger>,
  ) {
    const owners = [...bodies, ...triggers];
    const values: unknown[] = [];
    for (const owner of owners) {
      owner.validate();
      const scale = new Vector3().setFromMatrixScale(owner.matrixWorld);
      values.push(
        owner,
        owner.settingsVersion,
        owner instanceof RigidBody ? owner.materialVersion : 0,
        // Writeback decomposes body scales again every step; rounding drops that float noise.
        ...scale.toArray().map((value) => Math.round(value * 1e8) / 1e8),
      );
      for (const source of colliderSources(owner)) {
        for (
          let node: Object3D | null = source;
          node && node !== owner;
          node = node.parent
        )
          values.push(node, ...node.matrix.elements);
        if (source instanceof Collider) values.push(source.settingsVersion);
        if (source instanceof Mesh || source instanceof MeshCollider) {
          const { drawRange, morphAttributes } = source.geometry;
          values.push(
            geometryVersion(source.geometry),
            drawRange.start,
            drawRange.count,
            Object.keys(morphAttributes).length,
          );
        }
      }
    }
    for (const joint of joints)
      values.push(
        joint,
        joint.enabled,
        joint.collideConnected,
        joint.settingsVersion,
      );
    if (
      values.length === this.values.length &&
      values.every((value, index) => value === this.values[index])
    )
      return undefined;
    const scales = new Map<Object3D, Vector3>();
    const capture = (object: Object3D, scale: Vector3) => {
      const previous = this.scales.get(object);
      if (previous && previous.distanceTo(scale) > 1e-6)
        throw new Error(
          `Physics scale cannot change after backend initialization: ${object.name || object.type}; recreate the body`,
        );
      scales.set(object, previous ?? scale);
    };
    for (const owner of owners) {
      const colliders = owner.getColliders();
      const transform = splitTransform(owner.matrixWorld);
      capture(owner, transform.scale);
      const inverse = transform.pose.invert();
      for (const collider of colliders)
        capture(
          collider.source,
          splitTransform(inverse.clone().multiply(collider.matrixWorld)).scale,
        );
    }
    return { values, scales };
  }
  commit(change: { values: unknown[]; scales: Map<Object3D, Vector3> }): void {
    this.values = change.values;
    this.scales = change.scales;
  }
  remove(object: Object3D): void {
    object.traverse((child) => this.scales.delete(child));
  }
  clear(): void {
    this.scales.clear();
    this.values = [];
  }
}
