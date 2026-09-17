import { Group, type Object3D, type Object3DEventMap } from "three";
import { RigidBody } from "./body.js";
import { Collider, validateGroups, type CollisionGroups } from "./colliders.js";
import { constructLike } from "./construct.js";
import { cleanup } from "./cleanup.js";
import { validateShape } from "./shapes.js";
import { splitTransform } from "./transforms.js";
import { assertOwned, getDefaultWorld, type PhysicsWorld } from "./world.js";

export interface TriggerOptions {
  readonly world?: PhysicsWorld;
}
export interface TriggerEventMap extends Object3DEventMap {
  enter: { readonly body: RigidBody };
  exit: { readonly body: RigidBody };
}
export class Trigger extends Group<TriggerEventMap> {
  readonly world: PhysicsWorld;
  private isDisposed = false;
  private currentGroups?: CollisionGroups;
  private version = 0;

  constructor(options: TriggerOptions = {}) {
    super();
    this.world = options.world ?? getDefaultWorld();
    this.world.register(this);
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
    assertOwned(this.world, this);
    if (value) validateGroups(value);
    this.currentGroups = value && { ...value };
    this.version++;
    return this;
  }
  overlaps(body: RigidBody): boolean {
    assertOwned(this.world, body);
    return this.getOverlappingBodies().includes(body);
  }
  getOverlappingBodies(): RigidBody[] {
    assertOwned(this.world, this);
    return this.world.getOverlappingBodies(this);
  }
  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    cleanup(
      [() => this.world.unregister(this), () => this.removeFromParent()],
      "Trigger disposal failed",
    );
  }
  validate(): void {
    assertOwned(this.world, this);
    this.updateWorldMatrix(true, true);
    splitTransform(this.matrix, this.name || this.type);
    splitTransform(this.matrixWorld, this.name || this.type);
    for (let node: Object3D | null = this; node; node = node.parent) {
      if (Math.min(node.scale.x, node.scale.y, node.scale.z) <= 0)
        throw new Error("Trigger requires positive scale");
      if (node !== this && node instanceof Trigger)
        throw new Error("Nested triggers are not supported");
      if (node instanceof RigidBody) {
        assertOwned(this.world, node);
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
    assertOwned(this.world, this);
    const target = constructLike(this, [{ world: this.world }]);
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
  }
  override copy(source: this, recursive = true): this {
    assertOwned(this.world, this);
    assertOwned(source.world, source);
    super.copy(source, recursive);
    return this.setCollisionGroups(source.collisionGroups);
  }
}
