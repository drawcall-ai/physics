import { Group, Mesh, Matrix4, Vector3, type Object3D } from "three";
import {
  Collider,
  BoxCollider,
  SphereCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
  validateMaterial,
} from "./objects.js";
import { assertRigidTransform } from "./objects.js";
import { validateVector } from "./state.js";
import { splitTransform } from "./transforms.js";
import { autoShape, validateShape } from "./shapes.js";
import type { AutoColliders, Vec3, PhysicsMaterial } from "./objects.js";
import { getDefaultWorld } from "./world.js";
import type { PhysicsWorld, PhysicsVelocity } from "./world.js";

export interface RigidBodyOptions {
  readonly world?: PhysicsWorld;
  readonly type?: "dynamic" | "static" | "kinematic";
  readonly colliders?: AutoColliders;
  readonly mass?: number;
  readonly canSleep?: boolean;
  readonly centerOfMass?: Vec3;
  readonly diagonalInertia?: Vec3;
  readonly principalAxes?: readonly [number, number, number, number];
}
export class RigidBody extends Group {
  #disposed = false;
  #world: PhysicsWorld;
  get world(): PhysicsWorld {
    return this.#world;
  }
  #options: RigidBodyOptions;
  get options(): RigidBodyOptions {
    return this.#options;
  }
  #linearDamping = 0;
  #angularDamping = 0;
  #gravityScale = 1;
  #material?: PhysicsMaterial;
  #version = 0;
  #materialVersion = 0;
  get materialVersion(): number {
    return this.#materialVersion;
  }

  constructor(options: RigidBodyOptions = {}) {
    super();
    this.#world = options.world ?? getDefaultWorld();
    this.#options = Object.freeze({
      ...options,
      centerOfMass:
        options.centerOfMass && Object.freeze<Vec3>([...options.centerOfMass]),
      diagonalInertia:
        options.diagonalInertia &&
        Object.freeze<Vec3>([...options.diagonalInertia]),
      principalAxes:
        options.principalAxes &&
        Object.freeze<readonly [number, number, number, number]>([
          ...options.principalAxes,
        ]),
    });
    validateMass(this.options);
    this.world.register(this);
  }
  get settingsVersion(): number {
    return this.#version;
  }
  get linearDamping(): number {
    return this.#linearDamping;
  }
  get angularDamping(): number {
    return this.#angularDamping;
  }
  get gravityScale(): number {
    return this.#gravityScale;
  }
  get material(): PhysicsMaterial | undefined {
    return this.#material;
  }
  private assertLive(): void {
    if (this.disposed) throw new Error("Rigid body has been disposed");
  }
  setLinearDamping(value: number): this {
    this.assertLive();
    validateDamping(value);
    this.#linearDamping = value;
    this.#version++;
    return this;
  }
  setAngularDamping(value: number): this {
    this.assertLive();
    validateDamping(value);
    this.#angularDamping = value;
    this.#version++;
    return this;
  }
  setGravityScale(value: number): this {
    this.assertLive();
    if (!Number.isFinite(value))
      throw new Error("Gravity scale must be finite");
    this.#gravityScale = value;
    this.#version++;
    return this;
  }
  setMaterial(value: PhysicsMaterial | undefined): this {
    this.assertLive();
    if (value) validateMaterial(value);
    this.#material = value && Object.freeze({ ...value });
    this.#materialVersion++;
    return this;
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

  getVelocity(): PhysicsVelocity {
    return this.world.getVelocity(this);
  }
  setVelocity(value: Partial<PhysicsVelocity>): this {
    if (value.linear) validateVector(value.linear);
    if (value.angular) validateVector(value.angular);
    this.world.setVelocity(this, value);
    return this;
  }
  teleport(matrix: Matrix4): void {
    assertRigidTransform(matrix);
    this.world.teleport(this, matrix);
  }
  setKinematicTarget(matrix: Matrix4): void {
    if (this.options.type !== "kinematic")
      throw new Error("Kinematic targets require a kinematic body");
    assertRigidTransform(matrix);
    this.world.setKinematicTarget(this, matrix);
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    validateVector(impulse);
    if (point) validateVector(point);
    this.world.applyImpulse(this, impulse, point);
  }
  applyForce(force: Vector3, point?: Vector3): void {
    validateVector(force);
    if (point) validateVector(point);
    this.world.applyForce(this, force, point);
  }
  wake(): void {
    this.world.wake(this);
  }
  sleep(): void {
    this.world.sleep(this);
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
    if (!sameOptions(this.options, source.options))
      throw new Error("Rigid body copy requires matching immutable options");
    super.copy(source, recursive);
    this.setVelocity(source.getVelocity());
    this.setLinearDamping(source.linearDamping)
      .setAngularDamping(source.angularDamping)
      .setGravityScale(source.gravityScale)
      .setMaterial(source.material);
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
      let node: Object3D | null = object;
      while (node && node !== this) {
        if (Math.min(node.scale.x, node.scale.y, node.scale.z) <= 0)
          throw new Error(
            `Collider requires positive scale: ${node.name || node.type}`,
          );
        node = node.parent;
      }
      splitTransform(object.matrixWorld, object.name || object.type);
      let collider: Collider;
      if (object instanceof Collider) collider = object;
      else {
        const shape = autoShape(object, this);
        switch (shape.kind) {
          case "box":
            collider = new BoxCollider().setSize(shape.size);
            break;
          case "sphere":
            collider = new SphereCollider().setRadius(shape.radius);
            break;
          case "capsule":
            collider = new CapsuleCollider()
              .setRadius(shape.radius)
              .setLength(shape.length);
            break;
          case "cylinder":
            collider = new CylinderCollider()
              .setRadius(shape.radius)
              .setHeight(shape.height);
            break;
          case "mesh":
            collider = new MeshCollider({
              approximation: shape.approximation,
            }).setGeometry(shape.geometry);
            break;
        }
        object.matrixWorld.decompose(
          collider.position,
          collider.quaternion,
          collider.scale,
        );
        collider.source = object;
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
      return collider;
    });
    return colliders;
  }
  getMaterial(collider: Collider): Required<PhysicsMaterial> {
    const material = collider.material ?? this.material;
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
    splitTransform(this.matrix, this.name || this.type);
    splitTransform(this.matrixWorld, this.name || this.type);
    if (this.parent && this.options.type !== "static") {
      const { scale } = splitTransform(this.parent.matrixWorld);
      if (
        Math.abs(scale.x - scale.y) > 1e-6 ||
        Math.abs(scale.x - scale.z) > 1e-6
      )
        throw new Error("Moving bodies require uniform ancestor scale");
    }
    let parent: Object3D | null = this;
    while (parent) {
      if (Math.min(parent.scale.x, parent.scale.y, parent.scale.z) <= 0)
        throw new Error(
          `Body requires positive scale: ${parent.name || parent.type}`,
        );
      if (parent !== this && parent instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
      parent = parent.parent;
    }
  }
}
function validateDamping(value: number): void {
  if (!Number.isFinite(value) || value < 0)
    throw new Error("Damping must be finite and nonnegative");
}
function validateMass(options: RigidBodyOptions): void {
  if (
    options.mass !== undefined &&
    (!Number.isFinite(options.mass) || options.mass <= 0)
  )
    throw new Error("Body mass must be positive");
  if (options.centerOfMass && !options.centerOfMass.every(Number.isFinite))
    throw new Error("Center of mass must be finite");
  const inertia = options.diagonalInertia;
  if (
    inertia &&
    (!inertia.every((v) => Number.isFinite(v) && v > 0) ||
      inertia.some((v) => 2 * v > inertia[0] + inertia[1] + inertia[2] + 1e-10))
  )
    throw new Error(
      "Principal inertia must be positive and satisfy the triangle inequality",
    );
  if (
    options.principalAxes &&
    (!options.principalAxes.every(Number.isFinite) ||
      Math.abs(Math.hypot(...options.principalAxes) - 1) > 1e-6)
  )
    throw new Error("Principal axes must be a normalized quaternion");
}

function sameTuple(
  a: readonly number[] | undefined,
  b: readonly number[] | undefined,
): boolean {
  return a === undefined
    ? b === undefined
    : b !== undefined && a.length === b.length && a.every((v, i) => v === b[i]);
}
function sameOptions(a: RigidBodyOptions, b: RigidBodyOptions): boolean {
  return (
    (a.type ?? "dynamic") === (b.type ?? "dynamic") &&
    (a.colliders ?? "auto") === (b.colliders ?? "auto") &&
    a.mass === b.mass &&
    (a.canSleep ?? true) === (b.canSleep ?? true) &&
    sameTuple(a.centerOfMass, b.centerOfMass) &&
    sameTuple(a.diagonalInertia, b.diagonalInertia) &&
    sameTuple(a.principalAxes, b.principalAxes)
  );
}
