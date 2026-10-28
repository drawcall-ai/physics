import { Group, Matrix4, Vector3, type Object3D } from "three";
import {
  Collider,
  validateMaterial,
  validateGroups,
} from "../colliders/collider.js";
import { constructLike } from "../construct.js";
import { assertPositiveScale, splitTransform } from "../transforms.js";
import { colliderOf } from "../colliders/shapes.js";
import { colliderSources } from "../colliders/sources.js";
import { validateMass, type MassProperties } from "./mass.js";
import type {
  AutoColliders,
  PhysicsMaterial,
  CollisionGroups,
} from "../colliders/collider.js";
import { commandWorld, joinedWorld } from "../membership.js";
import type { PhysicsWorld } from "../world.js";
import type { Contact } from "../interactions.js";
import { validateDamping, type BodyEventMap } from "../body.js";
import {
  copyVelocity,
  initialVelocity,
  validateVelocity,
  type InitialVelocity,
  type PhysicsVelocity,
} from "./velocity.js";
import { SoftBody } from "../soft/body.js";

export type RigidBodyType = "dynamic" | "static" | "kinematic";

export type RigidBodyOptions = MassProperties & {
  readonly bodyType?: RigidBodyType;
  readonly colliders?: AutoColliders;
  readonly canSleep?: boolean;
  /** The velocity a dynamic body joins a world with and `reset()` returns it to; angular in rad/s. */
  readonly velocity?: Partial<InitialVelocity>;
  readonly material?: PhysicsMaterial;
};
type NormalizedOptions = RigidBodyOptions & {
  readonly bodyType: RigidBodyType;
  readonly colliders: AutoColliders;
  readonly canSleep: boolean;
  readonly velocity?: InitialVelocity;
};
const optionKeys: Record<keyof RigidBodyOptions, true> = {
  bodyType: true,
  colliders: true,
  canSleep: true,
  mass: true,
  centerOfMass: true,
  diagonalInertia: true,
  principalAxes: true,
  velocity: true,
  material: true,
};

export class RigidBody extends Group<BodyEventMap> {
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
    if (options.material) this.setMaterial(options.material);
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
  /** A rate in 1/s, independent of mass. */
  setLinearDamping(value: number): this {
    validateDamping(value);
    this.currentLinearDamping = value;
    this.version++;
    return this;
  }
  /** A rate in 1/s, independent of mass. */
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
  /** Velocity in m/s at a world-space point, including the body's rotation. */
  getVelocityAtPoint(point: Vector3): Vector3 {
    return commandWorld(this).getVelocityAtPoint(this, point);
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
  /** Adds a world-space torque in N·m for the next fixed step. */
  applyTorque(torque: Vector3): void {
    commandWorld(this).applyTorque(this, torque);
  }
  wake(): void {
    commandWorld(this).wake(this);
  }
  sleep(): void {
    commandWorld(this).sleep(this);
  }
  /** Contacts with other bodies in the last step, one per contact point. */
  getContacts(): Contact[] {
    return commandWorld(this).getContacts(this);
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
      torsionalFriction: material?.torsionalFriction ?? 0,
      rollingFriction: material?.rollingFriction ?? 0,
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
      if (
        node !== this &&
        (node instanceof RigidBody || node instanceof SoftBody)
      )
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

export function assertBodyType(
  body: RigidBody,
  type: RigidBodyType,
  subject: string,
): void {
  if (body.bodyType !== type)
    throw new Error(`${subject} requires a ${type} body`);
}
