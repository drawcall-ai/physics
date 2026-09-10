import { Group, Matrix4, Mesh } from "three";
import {
  Collider,
  assertRigidTransform,
  BoxCollider,
  SphereCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
} from "./objects.js";
import { autoShape, validateShape } from "./shapes.js";
import type { AutoColliders, Vec3, PhysicsMaterial } from "./objects.js";
import { getDefaultWorld } from "./world.js";
import type { PhysicsWorld } from "./world.js";

export interface RigidBodyOptions {
  readonly world?: PhysicsWorld;
  type?: "dynamic" | "static" | "kinematic";
  colliders?: AutoColliders;
  mass?: number;
  material?: PhysicsMaterial;
  linearDamping?: number;
  angularDamping?: number;
  gravityScale?: number;
  canSleep?: boolean;
  linearVelocity?: Vec3;
  angularVelocity?: Vec3;
}
export class RigidBody extends Group {
  #disposed = false;

  constructor(
    readonly options: RigidBodyOptions = {},
    readonly world: PhysicsWorld = options.world ?? getDefaultWorld(),
  ) {
    super();
    this.world.register(this);
  }

  get disposed(): boolean {
    return this.#disposed;
  }

  dispose(): void {
    if (this.#disposed) return;
    this.world.unregister(this);
    this.removeFromParent();
    this.#disposed = true;
  }

  override clone(recursive = true): this {
    if (this.disposed) throw new Error("Cannot clone a disposed rigid body");
    const target: unknown = Reflect.construct(this.constructor, [
      { ...this.options, world: this.world },
    ]);
    if (!this.isClone(target))
      throw new Error(
        "Rigid body clone constructor returned an incompatible object",
      );
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
  }

  private isClone(value: unknown): value is this {
    return (
      value instanceof RigidBody &&
      Object.getPrototypeOf(value) === Object.getPrototypeOf(this)
    );
  }

  override copy(source: this, recursive = true): this {
    if (this.disposed || source.disposed)
      throw new Error("Cannot copy a disposed rigid body");
    const options: RigidBodyOptions = {
      ...source.options,
      world: this.world,
      linearVelocity: source.options.linearVelocity
        ? [...source.options.linearVelocity]
        : undefined,
      angularVelocity: source.options.angularVelocity
        ? [...source.options.angularVelocity]
        : undefined,
    };
    super.copy(source, recursive);
    for (const key of Object.keys(this.options))
      Reflect.deleteProperty(this.options, key);
    Object.assign(this.options, options);
    return this;
  }

  getColliders(): Collider[] {
    this.validate();
    const explicit: Collider[] = [];
    const meshes: Mesh[] = [];
    this.traverse((object) => {
      if (object !== this && object instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
      if (object instanceof Collider) explicit.push(object);
      if (object instanceof Mesh) meshes.push(object);
    });
    const sources = explicit.length
      ? explicit
      : this.options.colliders === false
        ? []
        : meshes;
    const colliders = sources.map((object) => {
      assertRigidTransform(
        new Matrix4()
          .copy(this.matrixWorld)
          .invert()
          .multiply(object.matrixWorld),
      );
      let collider: Collider;
      if (object instanceof Collider) collider = object;
      else {
        const shape = autoShape(object, this);
        switch (shape.kind) {
          case "box":
            collider = new BoxCollider(shape);
            break;
          case "sphere":
            collider = new SphereCollider(shape);
            break;
          case "capsule":
            collider = new CapsuleCollider(shape);
            break;
          case "cylinder":
            collider = new CylinderCollider(shape);
            break;
          case "mesh":
            collider = new MeshCollider(shape);
            break;
        }
        object.matrixWorld.decompose(
          collider.position,
          collider.quaternion,
          collider.scale,
        );
        collider.name = object.name;
        collider.updateMatrixWorld(true);
      }
      const shape = collider.shape();
      validateShape(shape);
      if (
        shape.kind === "mesh" &&
        shape.approximation === "trimesh" &&
        this.options.type !== "static"
      )
        throw new Error("Triangle mesh colliders require static bodies");
      const material = this.getMaterial(collider);
      if (
        ![
          material.staticFriction,
          material.dynamicFriction,
          material.density,
        ].every((value) => Number.isFinite(value) && value >= 0) ||
        !Number.isFinite(material.restitution) ||
        material.restitution < 0 ||
        material.restitution > 1
      )
        throw new Error("Invalid physics material");
      const collisionGroups = collider.collisionGroups;
      if (
        collisionGroups &&
        ![collisionGroups.membership, collisionGroups.filter].every(
          (value) => Number.isInteger(value) && value >= 0 && value <= 65535,
        )
      )
        throw new Error("Collision groups must be unsigned 16-bit masks");
      return collider;
    });
    if (!colliders.length)
      throw new Error("Rigid body requires at least one collider");
    if (
      (this.options.type ?? "dynamic") === "dynamic" &&
      this.options.mass === undefined &&
      colliders.every((collider) => this.getMaterial(collider).density === 0)
    )
      throw new Error(
        "Dynamic body requires positive mass or collider density",
      );
    return colliders;
  }
  getMaterial(collider: Collider): Required<PhysicsMaterial> {
    const material = collider.material ?? this.options.material;
    return {
      staticFriction: material?.staticFriction ?? 0.5,
      dynamicFriction: material?.dynamicFriction ?? 0.5,
      restitution: material?.restitution ?? 0,
      density: material?.density ?? 1000,
    };
  }

  validate(): void {
    if (this.disposed) throw new Error("Cannot validate a disposed rigid body");
    this.updateWorldMatrix(true, true);
    assertRigidTransform(this.matrixWorld);
    let parent = this.parent;
    while (parent) {
      if (parent instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
      parent = parent.parent;
    }
    if (
      this.options.mass !== undefined &&
      (!Number.isFinite(this.options.mass) || this.options.mass <= 0)
    )
      throw new Error("Body mass must be positive");
    if (
      ![
        this.options.linearDamping ?? 0,
        this.options.angularDamping ?? 0,
      ].every((value) => Number.isFinite(value) && value >= 0)
    )
      throw new Error("Damping must be finite and nonnegative");
    if (
      ![
        this.options.gravityScale ?? 1,
        ...(this.options.linearVelocity ?? [0, 0, 0]),
        ...(this.options.angularVelocity ?? [0, 0, 0]),
      ].every(Number.isFinite)
    )
      throw new Error("Body velocities and gravity scale must be finite");
  }
}
