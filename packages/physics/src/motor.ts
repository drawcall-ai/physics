import { Matrix4, Vector3, type Object3D } from "three";
import type { CapsuleCastHit, CapsuleShape } from "./capsule.js";
import type { PhysicsWorld } from "./world.js";
import { RigidBody } from "./rigid/body.js";
import { SoftBody } from "./soft/body.js";
import { validateVector } from "./transforms.js";

const skin = 0.004;
const tolerance = 0.0001;
const minGroundNormal = Math.cos(Math.PI / 4);
const stepHeight = 0.32;

/**
 * Collision motion for an upright, world-Y capsule with a visual object's origin at its feet.
 * Dimensions are in world metres, independent of the visual object's rotation and scale.
 * Moves modify `position`; `commit()` writes it to the object. The motor adds no gravity,
 * velocity or step subscription, pushes no bodies, and participates in no native contacts.
 * It climbs steps up to 0.32 m and treats slopes up to 45° as walkable. Collision queries
 * inherit `PhysicsWorld`'s capsule-query limitations; spawn clear of obstacles and use `recover`
 * only for shallow overlap. A query failure throws and may leave a partially moved position.
 * @see PhysicsWorld.capsuleContacts
 * @see PhysicsWorld.castCapsule
 */
export class CapsuleMotor {
  /** Authoritative world-space position of the capsule's feet. Edit to reposition explicitly. */
  readonly position = new Vector3();
  private currentHeight: number;

  constructor(
    private readonly world: PhysicsWorld,
    private readonly object: Object3D,
    /** Capsule radius in world metres; positive and finite. */
    readonly radius: number,
    height: number,
  ) {
    if (!Number.isFinite(radius) || radius <= 0)
      throw new Error("Capsule radius must be positive and finite");
    this.validateHeight(height);
    this.currentHeight = height;
    if (object instanceof RigidBody || object instanceof SoftBody)
      throw new Error(
        "CapsuleMotor requires a visual object, not a physics body",
      );
    object.getWorldPosition(this.position);
    validateVector(this.position);
  }

  /** Finite total height, greater than twice the radius. Assignment bypasses clearance checks; use `resize` for growth. */
  get height(): number {
    return this.currentHeight;
  }
  set height(value: number) {
    this.validateHeight(value);
    this.currentHeight = value;
  }

  private validateHeight(height: number): void {
    if (!Number.isFinite(height) || height <= 2 * this.radius)
      throw new Error(
        "Capsule height must be finite and exceed twice its radius",
      );
  }

  private get shape(): CapsuleShape {
    return { radius: this.radius, height: this.height - 2 * this.radius };
  }

  private pose(height = this.height): Matrix4 {
    validateVector(this.position);
    return new Matrix4().makeTranslation(
      this.position.x,
      this.position.y + height / 2,
      this.position.z,
    );
  }

  /** Resizes at fixed feet; returns false when growth would overlap an obstacle. Throws leave height unchanged. */
  resize(height: number): boolean {
    this.validateHeight(height);
    if (
      height > this.height &&
      this.world
        .capsuleContacts(
          { radius: this.radius, height: height - 2 * this.radius },
          this.pose(height),
          0,
        )
        .some((contact) => contact.distance < -tolerance)
    )
      return false;
    this.currentHeight = height;
    return true;
  }

  /** Resolves shallow overlap, throwing if the bounded recovery cannot clear the capsule. */
  recover(): void {
    for (let i = 0; i < 8; i++) {
      const contacts = this.world.capsuleContacts(this.shape, this.pose(), 0);
      const deepest = contacts.sort((a, b) => a.distance - b.distance)[0];
      if (!deepest || deepest.distance >= -tolerance) return;
      this.position.addScaledVector(
        deepest.normal,
        Math.min(0.3, tolerance - deepest.distance),
      );
    }
    if (
      this.world
        .capsuleContacts(this.shape, this.pose(), 0)
        .some((contact) => contact.distance < -tolerance)
    )
      throw new Error(
        "Character capsule could not recover from overlapping geometry",
      );
  }

  /** Sweeps and slides by a world-space displacement; returns encountered normals. Enable stepping only while grounded. */
  move(displacement: Vector3, canStep: boolean): Vector3[] {
    const start = this.position.clone();
    const normals = this.slide(displacement);
    const horizontal = displacement.clone().setY(0);
    if (
      !canStep ||
      horizontal.lengthSq() < 1e-8 ||
      !normals.some((normal) => normal.y < minGroundNormal)
    )
      return normals;

    const sliding = this.position.clone();
    this.position.copy(start);
    this.slide(new Vector3(0, stepHeight, 0));
    if (this.position.y < start.y + stepHeight - skin * 2) {
      this.position.copy(sliding);
      return normals;
    }
    this.slide(horizontal);
    const landing = this.ground(stepHeight + 0.08);
    const progress = (position: Vector3) =>
      position.clone().sub(start).setY(0).dot(horizontal);
    if (
      !landing ||
      landing.point.y > start.y + stepHeight + skin ||
      progress(this.position) <= progress(sliding) + 1e-6 ||
      this.position.y <= start.y + 0.01 ||
      this.position.y > start.y + stepHeight + skin
    ) {
      this.position.copy(sliding);
      return normals;
    }
    return [landing.normal];
  }

  /** Carry by the actual support transform; its old surface may now overlap our starting pose. */
  carry(displacement: Vector3, support: RigidBody): void {
    this.slide(displacement, support);
  }

  private slide(displacement: Vector3, excluded?: RigidBody): Vector3[] {
    validateVector(this.position);
    validateVector(displacement);
    if (!Number.isFinite(displacement.length()))
      throw new Error("Capsule displacement length must be finite");
    const remaining = displacement.clone();
    const planes: Vector3[] = [];
    for (let i = 0; i < 8 && remaining.lengthSq() > 1e-10; i++) {
      const hit = this.world.castCapsule(
        this.shape,
        this.pose(),
        remaining,
        excluded ? { excludeBodies: [excluded] } : {},
      );
      if (!hit) {
        this.position.add(remaining);
        break;
      }
      const fraction = Math.max(0, hit.fraction - skin / remaining.length());
      this.position.addScaledVector(remaining, fraction);
      remaining.multiplyScalar(1 - fraction);
      planes.push(hit.normal);
      // Retain every plane: clipping only against the last hit re-enters acute corners.
      for (let pass = 0; pass < 4; pass++)
        for (const normal of planes) {
          const inward = remaining.dot(normal);
          if (inward < 0) remaining.addScaledVector(normal, -inward);
          if (normal.y > 0 && normal.y < minGroundNormal && displacement.y <= 0)
            remaining.y = Math.min(remaining.y, 0);
        }
    }
    return planes;
  }

  /** Casts downward and snaps to walkable support within `distance` metres; returns null without moving if none is found. */
  ground(distance: number): CapsuleCastHit | null {
    if (!Number.isFinite(distance) || distance < 0)
      throw new Error("Ground distance must be finite and nonnegative");
    const hit = this.world.castCapsule(
      this.shape,
      this.pose(),
      new Vector3(0, -distance, 0),
    );
    if (!hit || !this.walkable(hit)) return null;
    this.position.y -= Math.max(0, distance * hit.fraction - skin);
    return hit;
  }

  private walkable(hit: CapsuleCastHit): boolean {
    if (hit.point.y > this.position.y + stepHeight + skin) return false;
    if (hit.normal.y >= minGroundNormal) return true;
    if (hit.normal.y <= 0) return false;
    // A capsule's rounded contact normal at a tread edge is not the tread's surface normal.
    const origin = hit.point
      .clone()
      .addScaledVector(hit.normal.clone().setY(0), -0.005);
    origin.y += 0.02;
    const surface = this.world.raycast(origin, new Vector3(0, -1, 0), 0.04);
    if (
      surface?.kind !== "body" ||
      surface.body !== hit.body ||
      surface.normal.y < minGroundNormal
    )
      return false;
    hit.normal.copy(surface.normal);
    return true;
  }

  /** Writes the feet position to the visual object through its current parent transform. */
  commit(): void {
    validateVector(this.position);
    const local = this.position.clone();
    const parent = this.object.parent;
    if (parent) {
      parent.updateWorldMatrix(true, false);
      const determinant = parent.matrixWorld.determinant();
      if (!Number.isFinite(determinant) || determinant === 0)
        throw new Error(
          "Capsule motor parent must have an invertible transform",
        );
      parent.worldToLocal(local);
    }
    validateVector(local);
    this.object.position.copy(local);
    this.object.updateMatrix();
    this.object.updateMatrixWorld(true);
  }
}
