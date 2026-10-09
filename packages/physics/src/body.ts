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
  assertRigidTransform,
  setWorldPose,
  splitTransform,
  validateVector,
} from "./transforms.js";
import { colliderOf } from "./colliders/shapes.js";
import { colliderSources } from "./colliders/sources.js";
import { rollback } from "./cleanup.js";
import { validateMass, type MassProperties } from "./mass.js";
import type {
  AutoColliders,
  PhysicsMaterial,
  CollisionGroups,
} from "./colliders/collider.js";
import { assembly, hierarchyJoints } from "./joints/assembly.js";
import {
  requireWorld,
  type PhysicsVelocity,
  type PhysicsWorld,
} from "./world.js";

export type RigidBodyType = "dynamic" | "static" | "kinematic";

export type RigidBodyOptions = MassProperties & {
  readonly bodyType?: RigidBodyType;
  readonly colliders?: AutoColliders;
  readonly canSleep?: boolean;
};
type NormalizedOptions = RigidBodyOptions & {
  readonly bodyType: RigidBodyType;
  readonly colliders: AutoColliders;
  readonly canSleep: boolean;
};
export interface RigidBodyEventMap extends Object3DEventMap {
  contactbegin: { readonly otherBody: RigidBody };
  contactend: { readonly otherBody: RigidBody };
}
export class RigidBody extends Group<RigidBodyEventMap> {
  readonly bodyType: RigidBodyType;
  /** The world simulating this body; set by the world as the body enters and leaves its scene. */
  world: PhysicsWorld | undefined = undefined;
  /** Counts setting changes, so backends reconcile only what changed. */
  version = 0;
  /** The body's velocity outside a world, which a world starts it from as it joins. */
  readonly #velocity: PhysicsVelocity = {
    linear: new Vector3(),
    angular: new Vector3(),
  };
  #options: NormalizedOptions;
  private currentLinearDamping = 0;
  private currentAngularDamping = 0;
  private currentGravityScale = 1;
  private currentMaterial?: PhysicsMaterial;
  private currentGroups?: CollisionGroups;

  constructor(options: RigidBodyOptions = {}) {
    super();
    this.bodyType = options.bodyType ?? "dynamic";
    this.#options = {
      ...options,
      bodyType: this.bodyType,
      colliders: options.colliders ?? "auto",
      canSleep: options.canSleep ?? true,
    };
    validateMass(this.#options);
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
  private assertDynamic(subject: string): void {
    if (this.bodyType !== "dynamic")
      throw new Error(`${subject} requires a dynamic body`);
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
    if (this.world) return this.world.getVelocity(this);
    const { linear, angular } = this.#velocity;
    return { linear: linear.clone(), angular: angular.clone() };
  }
  setVelocity(value: Partial<PhysicsVelocity>): this {
    if (value.linear) validateVector(value.linear);
    if (value.angular) validateVector(value.angular);
    this.assertDynamic("Velocity");
    if (this.world) this.world.setVelocity(this, value);
    else {
      if (value.linear) this.#velocity.linear.copy(value.linear);
      if (value.angular) this.#velocity.angular.copy(value.angular);
    }
    return this;
  }
  /**
   * Moves the body, and every dynamic body jointed to it, rigidly to the pose. An assembly
   * articulated to a static or kinematic base or the world can only move within those joints.
   */
  teleport(matrix: Matrix4): this {
    assertRigidTransform(matrix);
    this.validate();
    const delta = matrix
      .clone()
      .multiply(splitTransform(this.matrixWorld).pose.invert());
    const poses = new Map(
      [...assembly(this, hierarchyJoints(this))].map((member) => {
        if (member === this) return [member, matrix];
        member.validate();
        return [
          member,
          delta.clone().multiply(splitTransform(member.matrixWorld).pose),
        ];
      }),
    );
    const restores = [...poses.keys()].map(saveTransform);
    for (const [member, pose] of poses) setWorldPose(member, pose);
    try {
      this.world?.teleport(this);
    } catch (error) {
      rollback(error, restores, "Teleport rollback failed");
    }
    return this;
  }
  /** Moves a kinematic body to the pose over the next step; outside a world it is placed there. */
  setKinematicTarget(matrix: Matrix4): this {
    if (this.bodyType !== "kinematic")
      throw new Error("Kinematic targets require a kinematic body");
    assertRigidTransform(matrix);
    if (this.world) this.world.setKinematicTarget(this, matrix);
    else setWorldPose(this, matrix);
    return this;
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    validateVector(impulse);
    if (point) validateVector(point);
    this.assertDynamic("An impulse");
    requireWorld(this).applyImpulse(this, impulse, point);
  }
  applyForce(force: Vector3, point?: Vector3): void {
    validateVector(force);
    if (point) validateVector(point);
    this.assertDynamic("A force");
    requireWorld(this).applyForce(this, force, point);
  }
  wake(): void {
    requireWorld(this).wake(this);
  }
  sleep(): void {
    requireWorld(this).sleep(this);
  }

  override clone(recursive = true): this {
    const target = constructLike(this, [this.options]);
    target.#options = this.#options;
    return target.copy(this, recursive);
  }

  /** Copies the settings of a body that shares these options, such as a clone. */
  override copy(source: this, recursive = true): this {
    if (source.options !== this.options)
      throw new Error("Rigid body copy requires the same immutable options");
    super.copy(source, recursive);
    if (this.bodyType === "dynamic") this.setVelocity(source.getVelocity());
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

/** Returns a function that puts the object's local transform back as it is now. */
function saveTransform(object: Object3D): () => void {
  const position = object.position.clone();
  const quaternion = object.quaternion.clone();
  const scale = object.scale.clone();
  return () => {
    object.position.copy(position);
    object.quaternion.copy(quaternion);
    object.scale.copy(scale);
    object.updateMatrix();
    object.updateMatrixWorld(true);
  };
}

function validateDamping(value: number): void {
  if (!Number.isFinite(value) || value < 0)
    throw new Error("Damping must be finite and nonnegative");
}
