import { BufferGeometry, Object3D } from "three";
import { constructLike } from "./construct.js";

export type Vec3 = readonly [number, number, number];
export type AutoColliders = "auto" | "box" | "convexHull" | "trimesh" | false;
export interface PhysicsMaterial {
  readonly staticFriction?: number;
  readonly dynamicFriction?: number;
  readonly restitution?: number;
  readonly density?: number;
}
export interface CollisionGroups {
  readonly membership: number;
  readonly filter: number;
}
export function validateMaterial(value: PhysicsMaterial): void {
  for (const [key, number] of Object.entries(value)) {
    if (number === undefined) continue;
    if (
      !Number.isFinite(number) ||
      number < 0 ||
      (key === "restitution" && number > 1)
    )
      throw new Error("Invalid physics material");
  }
}
export function validateGroups(value: CollisionGroups): void {
  if (
    ![value.membership, value.filter].every(
      (n) => Number.isInteger(n) && n >= 0 && n <= 65535,
    )
  )
    throw new Error("Collision groups must be unsigned 16-bit masks");
}
function positive(value: number): number {
  if (!Number.isFinite(value) || value <= 0)
    throw new Error("Collider dimensions must be positive and finite");
  return value;
}
export abstract class Collider extends Object3D {
  source: Object3D = this;
  private currentMaterial?: PhysicsMaterial;
  private currentGroups?: CollisionGroups;
  private version = 0;
  get settingsVersion(): number {
    return this.version;
  }
  get material(): PhysicsMaterial | undefined {
    return this.currentMaterial;
  }
  get collisionGroups(): CollisionGroups | undefined {
    return this.currentGroups;
  }
  setMaterial(value: PhysicsMaterial | undefined): this {
    if (value) validateMaterial(value);
    this.currentMaterial = value && { ...value };
    this.version++;
    return this;
  }
  setCollisionGroups(value: CollisionGroups | undefined): this {
    if (value) validateGroups(value);
    this.currentGroups = value && { ...value };
    this.version++;
    return this;
  }
  abstract shape(): Shape;
  override clone(recursive = true): this {
    return constructLike(this, [this.shape()]).copy(this, recursive);
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setMaterial(source.material).setCollisionGroups(
      source.collisionGroups,
    );
  }
}
export type Shape =
  | { kind: "box"; size: Vec3 }
  | { kind: "sphere"; radius: number }
  | { kind: "capsule"; radius: number; height: number }
  | { kind: "cylinder"; radius: number; height: number }
  | {
      kind: "mesh";
      geometry: BufferGeometry;
      approximation: "convexHull" | "trimesh";
    };
export class BoxCollider extends Collider {
  readonly size: Vec3;
  constructor(options: { readonly size?: Vec3 } = {}) {
    super();
    const size = options.size ?? [1, 1, 1];
    this.size = [positive(size[0]), positive(size[1]), positive(size[2])];
  }
  shape(): Shape {
    return { kind: "box", size: this.size };
  }
  override copy(source: this, recursive = true): this {
    if (!this.size.every((value, index) => value === source.size[index]))
      throw new Error("Cannot copy different immutable collider dimensions");
    return super.copy(source, recursive);
  }
}
export class SphereCollider extends Collider {
  readonly radius: number;
  constructor(options: { readonly radius?: number } = {}) {
    super();
    this.radius = positive(options.radius ?? 0.5);
  }
  shape(): Shape {
    return { kind: "sphere", radius: this.radius };
  }
  override copy(source: this, recursive = true): this {
    if (this.radius !== source.radius)
      throw new Error("Cannot copy different immutable collider dimensions");
    return super.copy(source, recursive);
  }
}
export class CapsuleCollider extends Collider {
  readonly radius: number;
  readonly height: number;
  constructor(
    options: { readonly radius?: number; readonly height?: number } = {},
  ) {
    super();
    this.radius = positive(options.radius ?? 0.5);
    this.height = positive(options.height ?? 1);
  }
  shape(): Shape {
    return { kind: "capsule", radius: this.radius, height: this.height };
  }
  override copy(source: this, recursive = true): this {
    if (this.radius !== source.radius || this.height !== source.height)
      throw new Error("Cannot copy different immutable collider dimensions");
    return super.copy(source, recursive);
  }
}
export class CylinderCollider extends Collider {
  readonly radius: number;
  readonly height: number;
  constructor(
    options: { readonly radius?: number; readonly height?: number } = {},
  ) {
    super();
    this.radius = positive(options.radius ?? 0.5);
    this.height = positive(options.height ?? 1);
  }
  shape(): Shape {
    return { kind: "cylinder", radius: this.radius, height: this.height };
  }
  override copy(source: this, recursive = true): this {
    if (this.radius !== source.radius || this.height !== source.height)
      throw new Error("Cannot copy different immutable collider dimensions");
    return super.copy(source, recursive);
  }
}
export class MeshCollider extends Collider {
  private currentGeometry = new BufferGeometry();
  readonly approximation: "convexHull" | "trimesh";
  constructor(
    options: { readonly approximation?: "convexHull" | "trimesh" } = {},
  ) {
    super();
    this.approximation = options.approximation ?? "convexHull";
  }
  get geometry(): BufferGeometry {
    return this.currentGeometry;
  }
  setGeometry(value: BufferGeometry): this {
    this.currentGeometry = value;
    return this;
  }
  shape(): Shape {
    return {
      kind: "mesh",
      geometry: this.geometry,
      approximation: this.approximation,
    };
  }
  override copy(source: this, recursive = true): this {
    if (source.approximation !== this.approximation)
      throw new Error("Cannot copy a different collider approximation");
    super.copy(source, recursive);
    return this.setGeometry(source.geometry);
  }
}

export function resolveCollisionGroups(
  collider: Collider,
  owner: { readonly collisionGroups?: CollisionGroups },
): CollisionGroups {
  return (
    collider.collisionGroups ??
    owner.collisionGroups ?? {
      membership: 0xffff,
      filter: 0xffff,
    }
  );
}
