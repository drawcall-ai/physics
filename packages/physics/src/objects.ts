import { BufferGeometry, Object3D, Matrix4, Quaternion, Vector3 } from "three";

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
  #material?: PhysicsMaterial;
  #groups?: CollisionGroups;
  #sensor = false;
  #version = 0;
  get settingsVersion(): number {
    return this.#version;
  }
  get material(): PhysicsMaterial | undefined {
    return this.#material;
  }
  get collisionGroups(): CollisionGroups | undefined {
    return this.#groups;
  }
  get sensor(): boolean {
    return this.#sensor;
  }
  setMaterial(value: PhysicsMaterial | undefined): this {
    if (value) validateMaterial(value);
    this.#material = value && Object.freeze({ ...value });
    this.#version++;
    return this;
  }
  setCollisionGroups(value: CollisionGroups | undefined): this {
    if (value) validateGroups(value);
    this.#groups = value && Object.freeze({ ...value });
    this.#version++;
    return this;
  }
  setSensor(value: boolean): this {
    if (typeof value !== "boolean") throw new Error("Sensor must be boolean");
    this.#sensor = value;
    this.#version++;
    return this;
  }
  abstract shape(): Shape;
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setMaterial(source.material)
      .setSensor(source.sensor)
      .setCollisionGroups(source.collisionGroups);
  }
}
export type Shape =
  | { kind: "box"; size: Vec3 }
  | { kind: "sphere"; radius: number }
  | { kind: "capsule"; radius: number; length: number }
  | { kind: "cylinder"; radius: number; height: number }
  | {
      kind: "mesh";
      geometry: BufferGeometry;
      approximation: "convexHull" | "trimesh";
    };
export class BoxCollider extends Collider {
  #size: Vec3 = Object.freeze([1, 1, 1]);
  get size(): Vec3 {
    return this.#size;
  }
  setSize(value: Vec3): this {
    value.forEach(positive);
    this.#size = Object.freeze([...value]);
    return this;
  }
  shape(): Shape {
    return { kind: "box", size: this.size };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setSize(source.size);
  }
}
export class SphereCollider extends Collider {
  #radius = 0.5;
  get radius(): number {
    return this.#radius;
  }
  setRadius(value: number): this {
    this.#radius = positive(value);
    return this;
  }
  shape(): Shape {
    return { kind: "sphere", radius: this.radius };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setRadius(source.radius);
  }
}
export class CapsuleCollider extends Collider {
  #radius = 0.5;
  get radius(): number {
    return this.#radius;
  }
  setRadius(value: number): this {
    this.#radius = positive(value);
    return this;
  }
  #length = 1;
  get length(): number {
    return this.#length;
  }
  setLength(value: number): this {
    this.#length = positive(value);
    return this;
  }
  override shape(): Shape {
    return { kind: "capsule", radius: this.radius, length: this.length };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setRadius(source.radius).setLength(source.length);
  }
}
export class CylinderCollider extends Collider {
  #radius = 0.5;
  get radius(): number {
    return this.#radius;
  }
  setRadius(value: number): this {
    this.#radius = positive(value);
    return this;
  }
  #height = 1;
  get height(): number {
    return this.#height;
  }
  setHeight(value: number): this {
    this.#height = positive(value);
    return this;
  }
  override shape(): Shape {
    return { kind: "cylinder", radius: this.radius, height: this.height };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setRadius(source.radius).setHeight(source.height);
  }
}
export class MeshCollider extends Collider {
  #geometry = new BufferGeometry();
  #approximation: "convexHull" | "trimesh";
  get approximation(): "convexHull" | "trimesh" {
    return this.#approximation;
  }
  constructor(
    options: { readonly approximation?: "convexHull" | "trimesh" } = {},
  ) {
    super();
    if (
      options.approximation !== undefined &&
      !["convexHull", "trimesh"].includes(options.approximation)
    )
      throw new Error("Invalid mesh collider approximation");
    this.#approximation = options.approximation ?? "convexHull";
  }
  get geometry(): BufferGeometry {
    return this.#geometry;
  }
  setGeometry(value: BufferGeometry): this {
    this.#geometry = value;
    return this;
  }
  shape(): Shape {
    return {
      kind: "mesh",
      geometry: this.geometry,
      approximation: this.approximation,
    };
  }
  override clone(recursive = true): this {
    const target: unknown = Reflect.construct(this.constructor, [
      { approximation: this.approximation },
    ]);
    if (!this.isClone(target)) throw new Error("Invalid collider clone");
    return target.copy(this, recursive);
  }
  private isClone(value: unknown): value is this {
    return (
      value instanceof MeshCollider &&
      Object.getPrototypeOf(value) === Object.getPrototypeOf(this)
    );
  }
  override copy(source: this, recursive = true): this {
    if (source.approximation !== this.approximation)
      throw new Error("Cannot copy a different collider approximation");
    super.copy(source, recursive);
    return this.setGeometry(source.geometry);
  }
}

export function assertRigidTransform(matrix: Matrix4): void {
  const position = new Vector3(),
    quaternion = new Quaternion(),
    scale = new Vector3();
  matrix.decompose(position, quaternion, scale);
  const rigid = new Matrix4().compose(
    position,
    quaternion,
    new Vector3(1, 1, 1),
  );
  if (
    !matrix.elements.every(
      (value, index) =>
        Number.isFinite(value) &&
        Math.abs(value - (rigid.elements[index] ?? Infinity)) < 1e-6,
    )
  ) {
    throw new Error("Physics transforms must have unit scale and no shear");
  }
}
