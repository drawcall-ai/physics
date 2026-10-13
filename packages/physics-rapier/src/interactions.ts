import type * as Rapier from "@dimforge/rapier3d-compat";
import { ancestorBody, type RigidBody, type Trigger } from "@drawcall/physics";
import { addContact, type Contacts } from "@drawcall/physics/backend";
import { Quaternion, Vector3 } from "three";
import type { Owner } from "./query.js";

/** The bodies each trigger overlaps after the last step. */
export function sampleOverlaps(
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
): Map<Trigger, Set<RigidBody>> {
  const overlaps = new Map<Trigger, Set<RigidBody>>();
  for (const [handle, owner] of owners) {
    if (owner.kind !== "trigger") continue;
    const parent = ancestorBody(owner.trigger);
    const bodies = overlaps.get(owner.trigger) ?? new Set<RigidBody>();
    overlaps.set(owner.trigger, bodies);
    simulation.intersectionPairsWith(
      simulation.getCollider(handle),
      (other) => {
        const hit = owners.get(other.handle);
        if (hit?.kind === "body" && hit.body !== parent) bodies.add(hit.body);
      },
    );
  }
  return overlaps;
}

/**
 * Each body's contact points in the last step: every manifold point, since Rapier rests bodies
 * slightly apart, with the normal impulse the solve gave it. Rapier 0.21 exposes no impulses
 * for triangle-mesh pairs, so their forces are 0, and points on a shared triangle edge repeat.
 */
export function sampleContacts(
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
): Contacts {
  const contacts: Contacts = new Map();
  for (const [handle, owner] of owners) {
    if (owner.kind !== "body") continue;
    const collider = simulation.getCollider(handle);
    // The points and normal are in the collider's local frame; its pose after the step places them.
    const rotation = new Quaternion().copy(collider.rotation());
    const origin = new Vector3().copy(collider.translation());
    simulation.contactPairsWith(collider, (other) => {
      const hit = owners.get(other.handle);
      if (hit?.kind !== "body" || hit.body === owner.body) return;
      simulation.contactPair(collider, other, (manifold, flipped) => {
        // This collider's outward normal; the other body pushes it the opposite way.
        const normal = new Vector3()
          .copy(flipped ? manifold.localNormal2() : manifold.localNormal1())
          .applyQuaternion(rotation)
          .negate();
        for (let i = 0; i < manifold.numContacts(); i++) {
          const local = flipped
            ? manifold.localContactPoint2(i)
            : manifold.localContactPoint1(i);
          if (!local) throw new Error("Missing Rapier contact point");
          addContact(contacts, owner.body, {
            otherBody: hit.body,
            point: new Vector3()
              .copy(local)
              .applyQuaternion(rotation)
              .add(origin),
            normal: normal.clone(),
            force: manifold.contactImpulse(i) / simulation.timestep,
          });
        }
      });
    });
  }
  return contacts;
}
