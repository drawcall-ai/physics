import { Object3D } from "three";
import { RigidBody } from "../rigid/body.js";
import type { Vec3 } from "../colliders/collider.js";
import { constructLike } from "../construct.js";
import { joinedWorld } from "../membership.js";
import type { PhysicsWorld } from "../world.js";
import { SoftBody } from "./body.js";

/** Simulation-node indices, or an inclusive box in the body's undeformed local coordinates. */
export type SoftRegion =
  readonly number[] | { readonly min: Vec3; readonly max: Vec3 };
export interface SoftAttachmentOptions {
  readonly body: SoftBody;
  readonly region: SoftRegion;
  readonly target: RigidBody | null;
}

/**
 * Clamps selected nodes at their initial offsets to a rigid body, or the world for a null target.
 * Keep mounts outside the target's collision surface: attachments do not suppress contact.
 */
export class SoftAttachment extends Object3D {
  readonly isPhysicsObject = true;
  readonly options: SoftAttachmentOptions;
  version = 0;
  private currentEnabled = true;
  constructor(options: SoftAttachmentOptions) {
    super();
    const region = options.region;
    if ("min" in region) {
      if (
        region.min.length !== 3 ||
        region.max.length !== 3 ||
        ![...region.min, ...region.max].every(Number.isFinite) ||
        region.min.some((v, i) => {
          const max = region.max[i];
          return max === undefined || v > max;
        })
      )
        throw new Error("Invalid soft attachment bounds");
      const min: Vec3 = [...region.min],
        max: Vec3 = [...region.max];
      this.options = Object.freeze({
        ...options,
        region: Object.freeze({
          min: Object.freeze(min),
          max: Object.freeze(max),
        }),
      });
    } else {
      if (!region.length || region.some((n) => !Number.isInteger(n) || n < 0))
        throw new Error(
          "Soft attachment needs nonnegative simulation-node indices",
        );
      this.options = Object.freeze({
        ...options,
        region: Object.freeze([...region]),
      });
    }
  }
  get world(): PhysicsWorld | undefined {
    return joinedWorld(this);
  }
  get enabled(): boolean {
    return this.currentEnabled;
  }
  setEnabled(value: boolean): this {
    this.currentEnabled = value;
    this.version++;
    return this;
  }
  override clone(): this {
    throw new Error("Clone attachments with their bodies using clone(root)");
  }
  cloneWithBodies(
    objects: ReadonlyMap<Object3D, Object3D>,
    recursive = true,
  ): this {
    const body = objects.get(this.options.body) ?? this.options.body;
    const target =
      this.options.target &&
      (objects.get(this.options.target) ?? this.options.target);
    if (
      !(body instanceof SoftBody) ||
      (target && !(target instanceof RigidBody))
    )
      throw new Error("Attachment clone requires the corresponding bodies");
    return constructLike(this, [{ ...this.options, body, target }]).copy(
      this,
      recursive,
    );
  }
  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    return this.setEnabled(source.enabled);
  }
}

export function selectSoftNodes(
  attachment: SoftAttachment,
  positions: ArrayLike<number>,
): number[] {
  const { region } = attachment.options;
  const count = positions.length / 3;
  if (!("min" in region)) {
    if (region.some((n) => n >= count))
      throw new Error("Soft attachment node is out of range");
    return [...new Set(region)];
  }
  const nodes: number[] = [];
  for (let i = 0; i < count; i++) {
    const p = [positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]];
    if (
      p.every((v, axis) => {
        const min = region.min[axis],
          max = region.max[axis];
        return (
          v !== undefined &&
          min !== undefined &&
          max !== undefined &&
          v >= min &&
          v <= max
        );
      })
    )
      nodes.push(i);
  }
  if (!nodes.length)
    throw new Error(
      `Soft attachment ${attachment.name || attachment.type} selects no simulation nodes`,
    );
  return nodes;
}
