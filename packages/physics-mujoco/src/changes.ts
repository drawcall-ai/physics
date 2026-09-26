import {
  RigidBody,
  splitTransform,
  geometryVersion,
  type Joint,
  type Trigger,
} from "@drawcall/physics";
import { type Object3D, Vector3 } from "three";

/** Compare authored data without constructing scaled collision geometry. */
export class Changes {
  private scales = new Map<Object3D, Vector3>();

  scan(
    bodies: Iterable<RigidBody>,
    joints: Iterable<Joint>,
    triggers: Iterable<Trigger>,
  ) {
    const scales = new Map<Object3D, Vector3>();
    const capture = (object: Object3D, scale: Vector3) => {
      const previous = this.scales.get(object);
      if (previous && previous.distanceTo(scale) > 1e-6)
        throw new Error(
          `Physics scale cannot change after backend initialization: ${object.name || object.type}; recreate the body`,
        );
      scales.set(object, previous ?? scale);
    };
    const owners = [...bodies, ...triggers].map((owner) => {
      const colliders = owner.getColliders();
      const transform = splitTransform(owner.matrixWorld);
      capture(owner, transform.scale);
      const inverse = transform.pose.invert();
      return [
        owner.id,
        owner.settingsVersion,
        owner instanceof RigidBody ? owner.materialVersion : 0,
        colliders.map((collider) => {
          const part = splitTransform(
            inverse.clone().multiply(collider.matrixWorld),
          );
          capture(collider.source, part.scale);
          const shape = collider.shape();
          return [
            collider.source.id,
            collider.settingsVersion,
            shape.kind === "mesh"
              ? [
                  shape.kind,
                  shape.approximation,
                  geometryVersion(shape.geometry),
                ]
              : shape,
            part.pose.elements.map((value) => Math.round(value * 1e8) / 1e8),
          ];
        }),
      ];
    });
    return {
      key: JSON.stringify([
        owners,
        [...joints].map((joint) => [
          joint.id,
          joint.enabled,
          joint.collideConnected,
          joint.settingsVersion,
        ]),
      ]),
      scales,
    };
  }
  commit(scales: Map<Object3D, Vector3>): void {
    this.scales = scales;
  }
  remove(object: Object3D): void {
    object.traverse((child) => this.scales.delete(child));
  }
  clear(): void {
    this.scales.clear();
  }
}
