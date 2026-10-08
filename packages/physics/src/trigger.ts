import { Group, type Object3D, type Object3DEventMap } from "three";
import { RigidBody, colliderSources } from "./body.js";
import { Collider, validateGroups, type CollisionGroups } from "./colliders.js";
import { constructLike } from "./construct.js";
import { validateShape } from "./shapes.js";
import { assertPositiveScale, assertScaledTransform } from "./transforms.js";
import { requireWorld } from "./worlds.js";

export interface TriggerEventMap extends Object3DEventMap {
  enter: { readonly body: RigidBody };
  exit: { readonly body: RigidBody };
}
export class Trigger extends Group<TriggerEventMap> {
  private currentGroups?: CollisionGroups;
  private version = 0;

  get settingsVersion(): number {
    return this.version;
  }
  get collisionGroups(): CollisionGroups | undefined {
    return this.currentGroups;
  }
  setCollisionGroups(value: CollisionGroups | undefined): this {
    if (value) validateGroups(value);
    this.currentGroups = value && { ...value };
    this.version++;
    return this;
  }
  overlaps(body: RigidBody): boolean {
    return this.getOverlappingBodies().includes(body);
  }
  getOverlappingBodies(): RigidBody[] {
    return requireWorld(this).getOverlappingBodies(this);
  }
  validate(): void {
    this.updateWorldMatrix(true, true);
    assertScaledTransform(this.matrix, this.name || this.type);
    assertScaledTransform(this.matrixWorld, this.name || this.type);
    for (let node: Object3D | null = this; node; node = node.parent) {
      assertPositiveScale(node, "Trigger");
      if (node !== this && node instanceof Trigger)
        throw new Error("Nested triggers are not supported");
      if (node instanceof RigidBody) node.validate();
    }
  }
  getColliders(): Collider[] {
    this.validate();
    const colliders = colliderSources(this).filter(
      (source) => source instanceof Collider,
    );
    for (const object of colliders) {
      for (
        let node: Object3D | null = object;
        node && node !== this;
        node = node.parent
      )
        assertPositiveScale(node, "Trigger shape");
      assertScaledTransform(object.matrixWorld, object.name || object.type);
      if (object.material !== undefined)
        throw new Error("Trigger colliders cannot have physics materials");
      const shape = object.shape();
      validateShape(shape);
      if (shape.kind === "mesh" && shape.approximation === "trimesh")
        throw new Error("Triangle meshes are not supported as trigger volumes");
    }
    return colliders;
  }
  override clone(recursive = true): this {
    return constructLike(this, []).copy(this, recursive);
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setCollisionGroups(source.collisionGroups);
  }
}
