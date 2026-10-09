import {
  Collider,
  GenericJoint,
  Joint,
  MeshCollider,
  ScalarJoint,
  jointDofs,
  splitTransform,
  type JointDrive,
  type RigidBody,
  type Trigger,
} from "@drawcall/physics";
import {
  colliderSources,
  geometryVersion,
  lockScale,
} from "@drawcall/physics/backend";
import { Mesh, type Object3D, Vector3 } from "three";

/** What a member's compiled model is made from; a different fingerprint needs a rebuild. */
export type Fingerprints = Map<RigidBody | Joint | Trigger, unknown[]>;

/**
 * The member's fingerprint, read without decomposing transforms: versions, an owner's world
 * scale, and its collider sources with the local transforms from them up to the owner, which
 * physics never writes. Moving bodies, and cameras or triggers moved inside them, read the same.
 */
export function fingerprint(member: RigidBody | Joint | Trigger): unknown[] {
  // Joint versions also count drive targets, which the model reads each step instead.
  if (member instanceof Joint)
    return [member.enabled, member.collideConnected, ...drives(member)];
  member.validate();
  const scale = new Vector3().setFromMatrixScale(member.matrixWorld);
  const values: unknown[] = [
    member.version,
    // Writeback decomposes body scales again every step; rounding drops that float noise.
    ...scale.toArray().map((value) => Math.round(value * 1e8) / 1e8),
  ];
  for (const source of colliderSources(member)) {
    for (
      let node: Object3D | null = source;
      node && node !== member;
      node = node.parent
    )
      values.push(node, ...node.matrix.elements);
    if (source instanceof Collider) values.push(source.version);
    if (source instanceof Mesh || source instanceof MeshCollider) {
      const { drawRange, morphAttributes } = source.geometry;
      values.push(
        geometryVersion(source.geometry),
        drawRange.start,
        drawRange.count,
        Object.keys(morphAttributes).length,
      );
    }
  }
  return values;
}

export function sameFingerprints(a: Fingerprints, b: Fingerprints): boolean {
  if (a.size !== b.size) return false;
  for (const [member, values] of a) {
    const other = b.get(member);
    if (
      !other ||
      other.length !== values.length ||
      values.some((value, i) => value !== other[i])
    )
      return false;
  }
  return true;
}

/**
 * The world scales of the owners and their collider sources, which must match those `locked`
 * holds: the compiled model bakes in an object's scale when it first holds it.
 */
export function lockScales(
  owners: Iterable<RigidBody | Trigger>,
  locked: ReadonlyMap<Object3D, Vector3>,
): Map<Object3D, Vector3> {
  const scales = new Map<Object3D, Vector3>();
  const lock = (object: Object3D, scale: Vector3) =>
    scales.set(
      object,
      lockScale(object.name || object.type, locked.get(object), scale),
    );
  for (const owner of owners) {
    const transform = splitTransform(owner.matrixWorld);
    lock(owner, transform.scale);
    const inverse = transform.pose.invert();
    for (const collider of owner.getColliders())
      lock(
        collider.source,
        splitTransform(inverse.clone().multiply(collider.matrixWorld)).scale,
      );
  }
  return scales;
}

/** The drives in a joint's slots; which drives they hold shapes the model. */
function drives(joint: Joint): (JointDrive | undefined)[] {
  if (joint instanceof ScalarJoint) return [joint.drive];
  if (joint instanceof GenericJoint)
    return jointDofs.map((axis) => joint.getDrive(axis));
  return [];
}
