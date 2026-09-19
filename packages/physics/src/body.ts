import {
  Group,
  Matrix4,
  Mesh,
  Vector3,
  type Object3D,
  type Object3DEventMap,
} from "three";
import { Collider, validateMaterial, validateGroups } from "./colliders.js";
import { constructLike } from "./construct.js";
import { cleanup } from "./cleanup.js";
import {
  assertRigidTransform,
  splitTransform,
  validateVector,
} from "./transforms.js";
import { Trigger } from "./trigger.js";
import { colliderOf } from "./shapes.js";
import type {
  AutoColliders,
  Vec3,
  PhysicsMaterial,
  CollisionGroups,
} from "./colliders.js";
import { registry } from "./registry.js";
import { authoredVelocity, setAuthoredVelocity } from "./velocity.js";
import { setWorldPose } from "./transforms.js";
import type { PhysicsVelocity } from "./world.js";

export type RigidBodyType = "dynamic" | "static" | "kinematic";

export type MassProperties =
  | {
      readonly mass?: number;
      readonly centerOfMass?: never;
      readonly diagonalInertia?: never;
      readonly principalAxes?: never;
    }
  | {
      readonly mass: number;
      readonly centerOfMass: Vec3;
      readonly diagonalInertia: Vec3;
      readonly principalAxes?: readonly [number, number, number, number];
    };
export type RigidBodyOptions = MassProperties & {
  readonly type?: RigidBodyType;
  readonly colliders?: AutoColliders;
  readonly canSleep?: boolean;
};
type NormalizedOptions = RigidBodyOptions & {
  readonly type: RigidBodyType;
  readonly colliders: AutoColliders;
  readonly canSleep: boolean;
};
export interface RigidBodyEventMap extends Object3DEventMap {
  contactbegin: { readonly otherBody: RigidBody };
  contactend: { readonly otherBody: RigidBody };
}
export class RigidBody extends Group<RigidBodyEventMap> {
  private isDisposed = false;
  readonly options: NormalizedOptions;
  readonly bodyType: RigidBodyType;
  private currentLinearDamping = 0;
  private currentAngularDamping = 0;
  private currentGravityScale = 1;
  private currentMaterial?: PhysicsMaterial;
  private currentGroups?: CollisionGroups;
  private version = 0;
  private materialEdits = 0;
  get materialVersion(): number {
    return this.materialEdits;
  }

  constructor(options: RigidBodyOptions = {}) {
    super();
    this.bodyType = options.type ?? "dynamic";
    const defaults = {
      type: this.bodyType,
      colliders: options.colliders ?? "auto",
      canSleep: options.canSleep ?? true,
    };
    this.options = options.centerOfMass
      ? {
          ...options,
          ...defaults,
          centerOfMass: [...options.centerOfMass],
          diagonalInertia: [...options.diagonalInertia],
          principalAxes: options.principalAxes && [...options.principalAxes],
        }
      : { ...options, ...defaults };
    validateMass(this.options);
    registry.register(this);
  }
  get settingsVersion(): number {
    return this.version;
  }
  get linearDamping(): number {
    return this.currentLinearDamping;
  }
  get angularDamping(): number {
    return this.currentAngularDamping;
  }
  get gravityScale(): number {
    return this.currentGravityScale;
  }
  get material(): PhysicsMaterial | undefined {
    return this.currentMaterial;
  }
  private assertLive(): void {
    if (this.disposed) throw new Error("Rigid body has been disposed");
  }
  setLinearDamping(value: number): this {
    this.assertLive();
    validateDamping(value);
    this.currentLinearDamping = value;
    this.version++;
    return this;
  }
  setAngularDamping(value: number): this {
    this.assertLive();
    validateDamping(value);
    this.currentAngularDamping = value;
    this.version++;
    return this;
  }
  setGravityScale(value: number): this {
    this.assertLive();
    if (!Number.isFinite(value))
      throw new Error("Gravity scale must be finite");
    this.currentGravityScale = value;
    this.version++;
    return this;
  }
  setMaterial(value: PhysicsMaterial | undefined): this {
    this.assertLive();
    if (value) validateMaterial(value);
    this.currentMaterial = value && { ...value };
    this.materialEdits++;
    return this;
  }
  get collisionGroups(): CollisionGroups | undefined {
    return this.currentGroups;
  }
  setCollisionGroups(value: CollisionGroups | undefined): this {
    this.assertLive();
    if (value) validateGroups(value);
    this.currentGroups = value && { ...value };
    this.version++;
    return this;
  }
  get disposed(): boolean {
    return this.isDisposed;
  }

  dispose(): void {
    if (this.isDisposed) return;
    this.isDisposed = true;
    const triggers: Trigger[] = [];
    this.traverse((object) => {
      if (object instanceof Trigger) triggers.push(object);
    });
    cleanup(
      [
        ...triggers.map((trigger) => () => trigger.dispose()),
        () => registry.unregister(this),
        () => this.removeFromParent(),
      ],
      "Rigid body disposal failed",
    );
  }

  getVelocity(): PhysicsVelocity {
    this.assertLive();
    return registry.world?.getVelocity(this) ?? authoredVelocity(this);
  }
  setVelocity(value: Partial<PhysicsVelocity>): this {
    if (value.linear) validateVector(value.linear);
    if (value.angular) validateVector(value.angular);
    this.assertLive();
    if (registry.world) registry.world.setVelocity(this, value);
    else setAuthoredVelocity(this, value);
    return this;
  }
  teleport(matrix: Matrix4): void {
    assertRigidTransform(matrix);
    this.validate();
    if (registry.world) registry.world.teleport(this, matrix);
    else setWorldPose(this, matrix);
  }
  setKinematicTarget(matrix: Matrix4): void {
    if (this.bodyType !== "kinematic")
      throw new Error("Kinematic targets require a kinematic body");
    assertRigidTransform(matrix);
    registry.requireWorld(this).setKinematicTarget(this, matrix);
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    validateVector(impulse);
    if (point) validateVector(point);
    registry.requireWorld(this).applyImpulse(this, impulse, point);
  }
  applyForce(force: Vector3, point?: Vector3): void {
    validateVector(force);
    if (point) validateVector(point);
    registry.requireWorld(this).applyForce(this, force, point);
  }
  wake(): void {
    registry.requireWorld(this).wake(this);
  }
  sleep(): void {
    registry.requireWorld(this).sleep(this);
  }

  override clone(recursive = true): this {
    if (this.disposed) throw new Error("Cannot clone a disposed rigid body");
    const target = constructLike(this, [this.options]);
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
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
      .setMaterial(source.material)
      .setCollisionGroups(source.collisionGroups);
    return this;
  }

  /** Explicit colliders take precedence; otherwise one is generated per visual mesh. */
  getColliders(): Collider[] {
    this.validate();
    const explicit: Collider[] = [];
    const meshes: Mesh[] = [];
    const collect = (object: Object3D): void => {
      if (object instanceof Trigger) return;
      if (object !== this && object instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
      if (object instanceof Collider) explicit.push(object);
      if (object instanceof Mesh) meshes.push(object);
      for (const child of object.children) collect(child);
    };
    collect(this);
    const sources: (Collider | Mesh)[] = explicit.length
      ? explicit
      : this.options.colliders === false
        ? []
        : meshes;
    return sources.map((source) => colliderOf(source, this));
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
    if (this.parent && this.bodyType !== "static") {
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

/** The rigid body an object sits under, if any. */
export function ancestorBody(object: Object3D): RigidBody | undefined {
  let parent = object.parent;
  while (parent && !(parent instanceof RigidBody)) parent = parent.parent;
  return parent ?? undefined;
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
    : b !== undefined && a.every((v, i) => v === b[i]);
}
function sameOptions(a: NormalizedOptions, b: NormalizedOptions): boolean {
  return (
    a.type === b.type &&
    a.colliders === b.colliders &&
    a.mass === b.mass &&
    a.canSleep === b.canSleep &&
    sameTuple(a.centerOfMass, b.centerOfMass) &&
    sameTuple(a.diagonalInertia, b.diagonalInertia) &&
    sameTuple(a.principalAxes ?? [0, 0, 0, 1], b.principalAxes ?? [0, 0, 0, 1])
  );
}
