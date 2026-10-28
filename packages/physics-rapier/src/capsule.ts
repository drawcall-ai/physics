import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  type CapsuleCastHit,
  type CapsuleContact,
  type CapsuleQueryOptions,
  type CapsuleShape,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { Owner } from "./query.js";

export function capsuleContacts(
  api: typeof Rapier,
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
  shape: CapsuleShape,
  pose: Matrix4,
  maxDistance: number,
  options: CapsuleQueryOptions,
): CapsuleContact[] {
  const capsule = new api.Capsule(shape.height / 2, shape.radius);
  const position = new Vector3().setFromMatrixPosition(pose);
  const rotation = new Quaternion().setFromRotationMatrix(pose);
  const contacts: CapsuleContact[] = [];
  for (const { collider, body, object } of obstacles(
    simulation,
    owners,
    options,
  )) {
    const contact = collider.contactShape(
      capsule,
      position,
      rotation,
      maxDistance,
    );
    if (!contact || contact.distance > maxDistance) continue;
    checkWitness(collider, contact);
    contacts.push({
      body,
      object,
      point: new Vector3().copy(contact.point1),
      normal: new Vector3().copy(contact.normal1),
      distance: contact.distance,
    });
  }
  return contacts;
}

export function castCapsule(
  api: typeof Rapier,
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
  shape: CapsuleShape,
  pose: Matrix4,
  displacement: Vector3,
  options: CapsuleQueryOptions,
): CapsuleCastHit | null {
  const length = displacement.length();
  if (length === 0) return null;
  const capsule = new api.Capsule(shape.height / 2, shape.radius);
  const position = new Vector3().setFromMatrixPosition(pose);
  const rotation = new Quaternion().setFromRotationMatrix(pose);
  const stationary = new Vector3();
  let closest: CapsuleCastHit | null = null;
  for (const { collider, body, object } of obstacles(
    simulation,
    owners,
    options,
  )) {
    const contact = collider.contactShape(capsule, position, rotation, length);
    if (contact) checkWitness(collider, contact);
    const approach = contact ? displacement.dot(contact.normal1) : 0;
    // A convex obstacle cannot block a path that stays on the other side of its
    // separating plane. This also rejects numerical sweep hits on tangent floors.
    // A mesh may have another blocking face, so it must still be swept.
    if (
      contact &&
      collider.shapeType() !== api.ShapeType.TriMesh &&
      (approach >= -length * 1e-8 || contact.distance + approach > 0)
    )
      continue;
    // Rapier may return a separated contact even when prediction is zero.
    if (contact && contact.distance <= 0 && approach < -length * 1e-8)
      return {
        body,
        object,
        point: new Vector3().copy(contact.point1),
        normal: new Vector3().copy(contact.normal1),
        distance: contact.distance,
        fraction: 0,
      };
    const hit = collider.castShape(
      stationary,
      capsule,
      position,
      rotation,
      displacement,
      0,
      closest?.fraction ?? 1,
      false,
    );
    if (!hit) continue;
    const colliderRotation = new Quaternion().copy(collider.rotation());
    closest = {
      body,
      object,
      point: new Vector3()
        .copy(hit.witness1)
        .applyQuaternion(colliderRotation)
        .add(collider.translation()),
      normal: new Vector3().copy(hit.normal1).applyQuaternion(colliderRotation),
      distance: hit.time_of_impact === 0 ? (contact?.distance ?? 0) : 0,
      fraction: hit.time_of_impact,
    };
  }
  return closest;
}

function checkWitness(
  collider: Rapier.Collider,
  contact: Rapier.ShapeContact,
): void {
  if (contact.distance !== 0) return;
  // Degenerate deep overlaps can report zero depth with an interior witness.
  // A genuine touching witness must lie on the obstacle surface, within 0.1 mm.
  const projection = collider.projectPoint(contact.point1, false);
  if (
    !projection ||
    new Vector3().copy(contact.point1).distanceTo(projection.point) > 1e-4
  )
    throw new Error(
      "Rapier capsule query could not resolve a penetration witness",
    );
}

function* obstacles(
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
  options: CapsuleQueryOptions,
) {
  for (const [handle, owner] of owners) {
    if (
      owner.kind !== "body" ||
      !(owner.body instanceof RigidBody) ||
      options.excludeBodies?.includes(owner.body)
    )
      continue;
    const collider = simulation.getCollider(handle);
    const groups = options.collisionGroups;
    const mask = collider.collisionGroups();
    if (
      groups &&
      (((mask >>> 16) & groups.filter) === 0 ||
        (mask & 0xffff & groups.membership) === 0)
    )
      continue;
    yield { collider, body: owner.body, object: owner.object };
  }
}
