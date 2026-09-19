import { Group, type Object3D, type Object3DEventMap } from "three";
import { RigidBody } from "./body.js";
import { Collider, validateGroups, type CollisionGroups } from "./colliders.js";
import { constructLike } from "./construct.js";
import { cleanup } from "./cleanup.js";
import { validateShape } from "./shapes.js";
import { splitTransform } from "./transforms.js";
import { registry } from "./registry.js";

export interface TriggerEventMap extends Object3DEventMap {
  enter: { readonly body: RigidBody };
  exit: { readonly body: RigidBody };
}
export class Trigger extends Group<TriggerEventMap> {
  private isDisposed = false;
  private currentGroups?: CollisionGroups;
  private version = 0;

  constructor() {
    super();
    registry.register(this);
  }
  get disposed(): boolean {
    return this.isDisposed;
  }
  get settingsVersion(): number {
    return this.version;
  }
  get collisionGroups(): CollisionGroups | undefined {
    return this.currentGroups;
  }
  setCollisionGroups(value: CollisionGroups | undefined): this {
    registry.assertRegistered(this);
    if (value) validateGroups(value);
    this.currentGroups = value && { ...value };
    this.version++;
    return this;
  }
  overlaps(body: RigidBody): boolean {
    registry.assertRegistered(body);
    return this.getOverlappingBodies().includes(body);
  }
  getOverlappingBodies(): RigidBody[] {
    return registry.requireWorld(this).getOverlappingBodies(this);
  }
  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    cleanup(
      [() => registry.unregister(this), () => this.removeFromParent()],
      "Trigger disposal failed",
    );
  }
  validate(): void {
    registry.assertRegistered(this);
    this.updateWorldMatrix(true, true);
    splitTransform(this.matrix, this.name || this.type);
    splitTransform(this.matrixWorld, this.name || this.type);
    for (let node: Object3D | null = this; node; node = node.parent) {
      if (Math.min(node.scale.x, node.scale.y, node.scale.z) <= 0)
        throw new Error("Trigger requires positive scale");
      if (node !== this && node instanceof Trigger)
        throw new Error("Nested triggers are not supported");
      if (node instanceof RigidBody) {
        registry.assertRegistered(node);
        node.validate();
      }
    }
  }
  getColliders(): Collider[] {
    this.validate();
    const colliders: Collider[] = [];
    this.traverse((object) => {
      if (
        object !== this &&
        (object instanceof Trigger || object instanceof RigidBody)
      )
        throw new Error("Triggers cannot contain triggers or rigid bodies");
      if (Math.min(object.scale.x, object.scale.y, object.scale.z) <= 0)
        throw new Error("Trigger shapes require positive scale");
      if (!(object instanceof Collider)) return;
      splitTransform(object.matrixWorld, object.name || object.type);
      if (object.material !== undefined)
        throw new Error("Trigger colliders cannot have physics materials");
      const shape = object.shape();
      validateShape(shape);
      if (shape.kind === "mesh" && shape.approximation === "trimesh")
        throw new Error("Triangle meshes are not supported as trigger volumes");
      colliders.push(object);
    });
    return colliders;
  }
  override clone(recursive = true): this {
    registry.assertRegistered(this);
    const target = constructLike(this, []);
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
  }
  override copy(source: this, recursive = true): this {
    registry.assertRegistered(this);
    registry.assertRegistered(source);
    super.copy(source, recursive);
    return this.setCollisionGroups(source.collisionGroups);
  }
}
