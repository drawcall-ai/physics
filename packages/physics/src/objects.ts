import { BufferGeometry, Object3D, Matrix4, Quaternion, Vector3 } from "three";

export type Vec3 = [number, number, number];
export type AutoColliders = "auto" | "box" | "convexHull" | "trimesh" | false;

export interface PhysicsMaterial {
  staticFriction?: number;
  dynamicFriction?: number;
  restitution?: number;
  density?: number;
}
export interface CollisionGroups {
  membership: number;
  filter: number;
}
export interface ColliderOptions {
  material?: PhysicsMaterial;
  sensor?: boolean;
  collisionGroups?: CollisionGroups;
}
export abstract class Collider extends Object3D {
  source: Object3D = this;
  collisionGroups?: CollisionGroups;
  material?: PhysicsMaterial;
  sensor = false;
  abstract shape(): Shape;
  constructor(options: ColliderOptions = {}) {
    super();
    Object.assign(this, options);
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.material = source.material;
    this.sensor = source.sensor;
    this.collisionGroups = source.collisionGroups
      ? { ...source.collisionGroups }
      : undefined;
    return this;
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
  size: Vec3;
  constructor(options: ColliderOptions & { size: Vec3 } = { size: [1, 1, 1] }) {
    super(options);
    this.size = [...options.size];
  }
  shape(): Shape {
    return { kind: "box", size: [...this.size] };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.size = [...source.size];
    return this;
  }
}
export class SphereCollider extends Collider {
  radius: number;
  constructor(options: ColliderOptions & { radius: number } = { radius: 0.5 }) {
    super(options);
    this.radius = options.radius;
  }
  shape(): Shape {
    return { kind: "sphere", radius: this.radius };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.radius = source.radius;
    return this;
  }
}
export class CapsuleCollider extends Collider {
  radius: number;
  length: number;
  constructor(
    options: ColliderOptions & { radius: number; length: number } = {
      radius: 0.5,
      length: 1,
    },
  ) {
    super(options);
    this.radius = options.radius;
    this.length = options.length;
  }
  shape(): Shape {
    return { kind: "capsule", radius: this.radius, length: this.length };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.radius = source.radius;
    this.length = source.length;
    return this;
  }
}
export class CylinderCollider extends Collider {
  radius: number;
  height: number;
  constructor(
    options: ColliderOptions & { radius: number; height: number } = {
      radius: 0.5,
      height: 1,
    },
  ) {
    super(options);
    this.radius = options.radius;
    this.height = options.height;
  }
  shape(): Shape {
    return { kind: "cylinder", radius: this.radius, height: this.height };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.radius = source.radius;
    this.height = source.height;
    return this;
  }
}
export class MeshCollider extends Collider {
  geometry: BufferGeometry;
  approximation: "convexHull" | "trimesh";
  constructor(
    options: ColliderOptions & {
      geometry?: BufferGeometry;
      approximation?: "convexHull" | "trimesh";
    } = {},
  ) {
    super(options);
    this.geometry = options.geometry ?? new BufferGeometry();
    this.approximation = options.approximation ?? "convexHull";
  }
  shape(): Shape {
    return {
      kind: "mesh",
      geometry: this.geometry,
      approximation: this.approximation,
    };
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.geometry = source.geometry;
    this.approximation = source.approximation;
    return this;
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
