import {
  RigidBody,
  splitTransform,
  type Joint,
  type Trigger,
} from "@drawcall/physics";
import { type BufferGeometry, type Object3D, Vector3 } from "three";

interface MeshSnapshot {
  positions: number[];
  indices: number[];
  version: number;
}

/** Compare authored data without constructing scaled collision geometry. */
export class Changes {
  private readonly meshes = new WeakMap<BufferGeometry, MeshSnapshot>();
  private scales = new Map<Object3D, Vector3>();
  private version = 0;

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
                  this.meshVersion(shape.geometry),
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
  private meshVersion(geometry: BufferGeometry): number {
    const position = geometry.getAttribute("position");
    const index = geometry.index;
    const previous = this.meshes.get(geometry);
    const positions = position.count * 3;
    const indices = index?.count ?? 0;
    let equal =
      previous?.positions.length === positions &&
      previous.indices.length === indices;
    for (let i = 0; equal && i < position.count; i++)
      equal =
        previous?.positions[i * 3] === position.getX(i) &&
        previous.positions[i * 3 + 1] === position.getY(i) &&
        previous.positions[i * 3 + 2] === position.getZ(i);
    for (let i = 0; equal && i < indices; i++)
      equal = previous?.indices[i] === index?.getX(i);
    if (equal && previous) return previous.version;
    const snapshot: MeshSnapshot = {
      positions: [],
      indices: [],
      version: ++this.version,
    };
    for (let i = 0; i < position.count; i++)
      snapshot.positions.push(
        position.getX(i),
        position.getY(i),
        position.getZ(i),
      );
    for (let i = 0; i < indices; i++) {
      if (!index) throw new Error("Missing mesh index");
      snapshot.indices.push(index.getX(i));
    }
    this.meshes.set(geometry, snapshot);
    return snapshot.version;
  }
}
