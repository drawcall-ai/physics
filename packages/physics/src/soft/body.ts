import {
  Group,
  Mesh,
  Vector3,
  BufferGeometry,
  Float32BufferAttribute,
  type Object3D,
} from "three";
import { RigidBody } from "../rigid/body.js";
import { validateDamping, type BodyEventMap } from "../body.js";
import { commandWorld, joinedWorld } from "../membership.js";
import { constructLike } from "../construct.js";
import { splitTransform, assertPositiveScale } from "../transforms.js";
import type { PhysicsWorld } from "../world.js";
import type { Contact } from "../interactions.js";
import { type SoftMaterial, copySoftMaterial } from "./material.js";
import { type SoftShape, softGeometry } from "./shape.js";
import type { ReducedModel } from "./model.js";
import { bindSoftGeometry, copySoftGeometry } from "./render.js";

export interface SoftShapeOptions {
  readonly shape: SoftShape;
  readonly material: SoftMaterial;
  readonly selfCollision?: boolean;
}
export type SoftBodyOptions =
  SoftShapeOptions | { readonly model: ReducedModel };

/** A fixed-topology deformable. Add a Mesh using `body.geometry` to render its deformation. */
export class SoftBody extends Group<BodyEventMap> {
  readonly isPhysicsObject = true;
  #options: SoftBodyOptions;
  /** Instance-owned render geometry. Rest geometry remains in the shape or reduced model. */
  readonly geometry: BufferGeometry;
  version = 0;
  private currentMaterial: SoftMaterial;
  private currentLinearDamping = 0;

  constructor(options: SoftBodyOptions) {
    super();
    const reduced = "model" in options;
    for (const key of Object.keys(options))
      if (
        !(
          reduced ? ["model"] : ["shape", "material", "selfCollision"]
        ).includes(key)
      )
        throw new Error(`Unknown soft-body option: ${key}`);
    if ("model" in options) {
      this.geometry = new BufferGeometry();
      this.geometry.setAttribute(
        "position",
        new Float32BufferAttribute(options.model.renderPositions, 3),
      );
      this.geometry.setIndex([...options.model.renderTriangles]);
      this.geometry.computeVertexNormals();
      this.currentMaterial = options.model.material;
      this.#options = Object.freeze({ model: options.model });
      return;
    }
    const shape = copyShape(options.shape);
    this.geometry = softGeometry(shape);
    this.currentMaterial = copySoftMaterial(options.material);
    this.#options = Object.freeze({
      ...options,
      shape,
      material: this.currentMaterial,
    });
  }
  get options(): SoftBodyOptions {
    return this.#options;
  }
  get model(): ReducedModel | undefined {
    return "model" in this.#options ? this.#options.model : undefined;
  }
  /**
   * Bind an attached visual mesh. Its topology, hierarchy and local transform stay fixed;
   * the solver owns its positions. External buffer edits use three.js's needsUpdate flag.
   */
  bindGeometry(mesh: Mesh, vertices: ArrayLike<number>): this {
    bindSoftGeometry(this, mesh, vertices);
    return this;
  }
  get world(): PhysicsWorld | undefined {
    return joinedWorld(this);
  }
  get material(): SoftMaterial {
    return this.currentMaterial;
  }
  get linearDamping(): number {
    return this.currentLinearDamping;
  }
  /**
   * Mass-proportional drag rate in 1/s: each node receives F = -rate * mass * velocity.
   * Defaults to zero. Unlike material damping, this damps bulk translation and rotation too.
   * Free velocity approaches v(t) = v(0) exp(-rate * t) as the timestep decreases.
   * This linear drag model is not an aerodynamic drag coefficient or measured material property.
   */
  setLinearDamping(value: number): this {
    validateDamping(value);
    this.currentLinearDamping = value;
    this.version++;
    return this;
  }
  /**
   * Replaces the complete material; supported edits preserve deformation and velocity.
   * Reduced FEM density and elasticity are baked; only contact properties may change without preparation.
   */
  setMaterial(value: SoftMaterial): this {
    const material = copySoftMaterial(value);
    if (
      this.model &&
      (material.density !== this.model.material.density ||
        !sameElasticity(material.elasticity, this.model.material.elasticity))
    )
      throw new Error(
        "Reduced FEM density and elasticity are baked; prepare a new model",
      );
    this.currentMaterial = material;
    this.version++;
    return this;
  }
  /** Copies the simulation nodes in world coordinates; these may differ from render vertices. */
  getPositions(): Float32Array {
    return commandWorld(this).getSoftPositions(this);
  }
  /** Without a point the load is distributed by mass; a point selects the nearest simulation node. */
  applyForce(force: Vector3, point?: Vector3): void {
    commandWorld(this).applyForce(this, force, point);
  }
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    commandWorld(this).applyImpulse(this, impulse, point);
  }
  getContacts(): Contact[] {
    return commandWorld(this).getContacts(this);
  }

  validate(): void {
    this.updateWorldMatrix(true, true);
    for (let node: Object3D | null = this; node; node = node.parent) {
      assertPositiveScale(node, "Soft body");
      if (
        node !== this &&
        (node instanceof SoftBody || node instanceof RigidBody)
      )
        throw new Error("Physics bodies cannot be nested; use an attachment");
    }
    const { scale } = splitTransform(this.matrixWorld);
    if (this.model && scale.distanceTo(new Vector3(1, 1, 1)) > 1e-6)
      throw new Error("Reduced FEM scale is baked; prepare a new model");
    if (
      Math.abs(scale.x - scale.y) > 1e-6 ||
      Math.abs(scale.y - scale.z) > 1e-6
    )
      throw new Error("Soft bodies require uniform scale");
  }

  /** Clones start at rest with independent deformed geometry; reduced mechanics and visual materials are shared. */
  override clone(recursive = true): this {
    const target = constructLike(this, [
      "model" in this.options
        ? this.options
        : { ...this.options, material: this.material },
    ]);
    target.#options = this.#options;
    return target.copy(this, recursive);
  }
  override copy(source: this, recursive = true): this {
    if (source === this) return this;
    const compatible =
      "model" in source.options
        ? "model" in this.options && source.options.model === this.options.model
        : "shape" in this.options &&
          source.options.shape === this.options.shape;
    if (!compatible)
      throw new Error(
        "Soft-body copy requires the same immutable shape; use clone()",
      );
    super.copy(source, recursive);
    this.setMaterial(source.material).setLinearDamping(source.linearDamping);
    this.traverse((child) => {
      if (child instanceof Mesh && child.geometry === source.geometry)
        child.geometry = this.geometry;
    });
    copySoftGeometry(source, this);
    return this;
  }
}

function sameElasticity(
  a: SoftMaterial["elasticity"],
  b: SoftMaterial["elasticity"],
): boolean {
  const left = Object.entries(a).filter(([, value]) => value !== undefined);
  const right = Object.entries(b).filter(([, value]) => value !== undefined);
  return (
    left.length === right.length &&
    left.every(([key, value]) =>
      right.some(([other, entry]) => key === other && value === entry),
    )
  );
}

function copyShape(shape: SoftShape): SoftShape {
  if (!["curve", "surface", "volume"].includes(shape.kind))
    throw new Error("Unknown soft-body shape kind");
  if (shape.kind === "curve")
    return Object.freeze({
      ...shape,
      points: Object.freeze(
        shape.points.map((p) => {
          const point: readonly [number, number, number] = [p[0], p[1], p[2]];
          return Object.freeze(point);
        }),
      ),
    });
  if ("geometry" in shape)
    return Object.freeze({ ...shape, geometry: shape.geometry.clone() });
  return Object.freeze({
    ...shape,
    positions: new Float32Array(shape.positions),
    tetrahedra: Object.freeze(Array.from(shape.tetrahedra)),
  });
}
