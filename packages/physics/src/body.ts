import {
  Group,
  Matrix4,
  Vector3,
  type Object3D,
  type Object3DEventMap,
} from "three";
import {
  Collider,
  validateMaterial,
  validateGroups,
} from "./colliders/collider.js";
import { constructLike } from "./construct.js";
import {
  assertPositiveScale,
  splitTransform,
  validateVector,
} from "./transforms.js";
import { colliderOf } from "./colliders/shapes.js";
import { colliderSources } from "./colliders/sources.js";
import { validateMass, type MassProperties } from "./mass.js";
import type {
  AutoColliders,
  PhysicsMaterial,
  CollisionGroups,
  Vec3,
} from "./colliders/collider.js";
import { commandWorld, joinedWorld } from "./membership.js";
import type { PhysicsVelocity, PhysicsWorld } from "./world.js";

export type RigidBodyType = "dynamic" | "static" | "kinematic";

export type RigidBodyOptions = MassProperties & {
  readonly bodyType?: RigidBodyType;
  readonly colliders?: AutoColliders;
  readonly canSleep?: boolean;
  /** The velocity a dynamic body joins a world with and `reset()` returns it to; angular in rad/s. */
  readonly velocity?: Partial<InitialVelocity>;
};
export interface InitialVelocity {
  readonly linear: Vec3;
  readonly angular: Vec3;
}
type NormalizedOptions = RigidBodyOptions & {
  readonly bodyType: RigidBodyType;
  readonly colliders: AutoColliders;
  readonly canSleep: boolean;
  readonly velocity?: InitialVelocity;
};
export interface RigidBodyEventMap extends Object3DEventMap {
  contactbegin: { readonly otherBody: RigidBody };
  contactend: { readonly otherBody: RigidBody };
}
const optionKeys: Record<keyof RigidBodyOptions, true> = {
  bodyType: true,
  colliders: true,
  canSleep: true,
  mass: true,
  centerOfMass: true,
  diagonalInertia: true,
  principalAxes: true,
  velocity: true,
};

export class RigidBody extends Group<RigidBodyEventMap> {
  readonly isPhysicsObject = true;
  readonly bodyType: RigidBodyType;
  /** Counts setting changes, so backends reconcile only what changed. */
  version = 0;
  #options: NormalizedOptions;
  private currentLinearDamping = 0;
  private currentAngularDamping = 0;
  private currentGravityScale = 1;
  private currentMaterial?: PhysicsMaterial;
  private currentGroups?: CollisionGroups;

  constructor(options: RigidBodyOptions = {}) {
    super();
    for (const key of Object.keys(options))
      if (!Object.hasOwn(optionKeys, key))
        throw new Error(`Unknown rigid body option: ${key}`);
    this.bodyType = options.bodyType ?? "dynamic";
    this.#options = {
      ...options,
      bodyType: this.bodyType,
      colliders: options.colliders ?? "auto",
      canSleep: options.canSleep ?? true,
      velocity: options.velocity && copyVelocity(options.velocity),
    };
    validateMass(this.#options);
    if (options.velocity) validateVelocity(this, initialVelocity(this));
  }
  /** The world simulating this body, while that world's scene holds it. */
  get world(): PhysicsWorld | undefined {
    return joinedWorld(this);
  }
  /** Fixed at construction and shared with clones. */
  get options(): NormalizedOptions {
    return this.#options;
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
  setLinearDamping(value: number): this {
    validateDamping(value);
    this.currentLinearDamping = value;
    this.version++;
    return this;
  }
  setAngularDamping(value: number): this {
    validateDamping(value);
    this.currentAngularDamping = value;
    this.version++;
    return this;
  }
  setGravityScale(value: number): this {
    if (!Number.isFinite(value))
      throw new Error("Gravity scale must be finite");
    this.currentGravityScale = value;
    this.version++;
    return this;
  }
  setMaterial(value: PhysicsMaterial | undefined): this {
    if (value) validateMaterial(value);
    this.currentMaterial = value && { ...value };
    this.version++;
    return this;
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
  getVelocity(): PhysicsVelocity {
    return commandWorld(this).getVelocity(this);
  }
  setVelocity(value: Partial<PhysicsVelocity>): this {
    commandWorld(this).setVelocity(this, value);
    return this;
  }
  /**
   * Moves the body, and every dynamic body jointed to it, rigidly to the pose. An assembly
   * articulated to a static or kinematic base or the world can only move within those joints.
   */
  teleport(pose: Matrix4): this {
    commandWorld(this).teleport(this, pose);
    return this;
  }
  /** Moves a kinematic body to the pose over the next step. */
  setKinematicTarget(pose: Matrix4): this {
    commandWorld(this).setKinematicTarget(this, pose);
    return this;
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    commandWorld(this).applyImpulse(this, impulse, point);
  }
  applyForce(force: Vector3, point?: Vector3): void {
    commandWorld(this).applyForce(this, force, point);
  }
  wake(): void {
    commandWorld(this).wake(this);
  }
  sleep(): void {
    commandWorld(this).sleep(this);
  }

  override clone(recursive = true): this {
    const target = constructLike(this, [this.options]);
    target.#options = this.#options;
    return target.copy(this, recursive);
  }

  /** Copies the settings of a body that shares these options, such as a clone. */
  override copy(source: this, recursive = true): this {
    if (source === this) return this;
    if (source.options !== this.options)
      throw new Error("Rigid body copy requires the same immutable options");
    super.copy(source, recursive);
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
    return colliderSources(this).map((source) => colliderOf(source, this));
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
    for (let node: Object3D | null = this; node; node = node.parent) {
      assertPositiveScale(node, "Body");
      if (node !== this && node instanceof RigidBody)
        throw new Error("Nested rigid bodies are not supported");
    }
  }
}

/** The rigid body an object sits under, if any. */
export function ancestorBody(object: Object3D): RigidBody | undefined {
  let parent = object.parent;
  while (parent && !(parent instanceof RigidBody)) parent = parent.parent;
  return parent ?? undefined;
}

const rest: InitialVelocity = { linear: [0, 0, 0], angular: [0, 0, 0] };

/** The velocity the body's options give it as it joins a world. */
export function initialVelocity(body: RigidBody): PhysicsVelocity {
  const { linear, angular } = body.options.velocity ?? rest;
  return { linear: new Vector3(...linear), angular: new Vector3(...angular) };
}

/** A frozen copy with both parts; rejects unknown keys and parts without three components. */
function copyVelocity(value: Partial<InitialVelocity>): InitialVelocity {
  for (const key of Object.keys(value))
    if (key !== "linear" && key !== "angular")
      throw new Error(`Unknown rigid body velocity option: ${key}`);
  const copy: InitialVelocity = {
    linear: copyVec3(value.linear ?? rest.linear),
    angular: copyVec3(value.angular ?? rest.angular),
  };
  Object.freeze(copy);
  return copy;
}

function copyVec3(value: Vec3): Vec3 {
  if (value.length !== 3)
    throw new Error("Velocity parts must have three components");
  const copy: Vec3 = [value[0], value[1], value[2]];
  Object.freeze(copy);
  return copy;
}

export function validateVelocity(
  body: RigidBody,
  value: Partial<PhysicsVelocity>,
): void {
  if (value.linear) validateVector(value.linear);
  if (value.angular) validateVector(value.angular);
  assertBodyType(body, "dynamic", "Velocity");
}

export function assertBodyType(
  body: RigidBody,
  type: RigidBodyType,
  subject: string,
): void {
  if (body.bodyType !== type)
    throw new Error(`${subject} requires a ${type} body`);
}

function validateDamping(value: number): void {
  if (!Number.isFinite(value) || value < 0)
    throw new Error("Damping must be finite and nonnegative");
}
