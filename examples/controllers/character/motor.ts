import { Matrix4, Vector3, type Object3D } from "three";
import type {
  CapsuleCastHit,
  CapsuleShape,
  PhysicsWorld,
  RigidBody,
} from "@drawcall/physics";

const skin = 0.004;
const tolerance = 0.0001;
const minGroundNormal = Math.cos(Math.PI / 4);
const stepHeight = 0.32;

/** Geometry-only motor. The capsule pose is authoritative; no native rigid body drives it. */
export class CapsuleMotor {
  readonly position = new Vector3();
  height: number;

  constructor(
    private readonly world: PhysicsWorld,
    private readonly object: Object3D,
    readonly radius: number,
    height: number,
  ) {
    this.height = height;
    object.getWorldPosition(this.position);
  }

  private get shape(): CapsuleShape {
    return { radius: this.radius, height: this.height - 2 * this.radius };
  }

  private get pose(): Matrix4 {
    return new Matrix4().makeTranslation(
      this.position.x,
      this.position.y + this.height / 2,
      this.position.z,
    );
  }

  resize(height: number): boolean {
    const previous = this.height;
    this.height = height;
    if (
      height > previous &&
      this.world
        .capsuleContacts(this.shape, this.pose, 0)
        .some((contact) => contact.distance < -tolerance)
    ) {
      this.height = previous;
      return false;
    }
    return true;
  }

  recover(): void {
    for (let i = 0; i < 8; i++) {
      const contacts = this.world.capsuleContacts(this.shape, this.pose, 0);
      const deepest = contacts.sort((a, b) => a.distance - b.distance)[0];
      if (!deepest || deepest.distance >= -tolerance) return;
      this.position.addScaledVector(
        deepest.normal,
        Math.min(0.3, tolerance - deepest.distance),
      );
    }
    if (
      this.world
        .capsuleContacts(this.shape, this.pose, 0)
        .some((contact) => contact.distance < -tolerance)
    )
      throw new Error(
        "Character capsule could not recover from overlapping geometry",
      );
  }

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
    const remaining = displacement.clone();
    const planes: Vector3[] = [];
    for (let i = 0; i < 8 && remaining.lengthSq() > 1e-10; i++) {
      const hit = this.world.castCapsule(
        this.shape,
        this.pose,
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

  ground(distance: number): CapsuleCastHit | null {
    const hit = this.world.castCapsule(
      this.shape,
      this.pose,
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

  commit(): void {
    this.object.position.copy(this.position);
    this.object.parent?.worldToLocal(this.object.position);
    this.object.updateMatrixWorld(true);
  }
}
