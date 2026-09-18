import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  RigidBody,
  Trigger,
  ancestorBody,
  Interactions,
} from "@drawcall/physics";
import type { BodyBinding } from "./body.js";
import type { TriggerBinding } from "./triggers.js";

export function sampleInteractions(
  interactions: Interactions,
  backend: Rapier.World,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
  triggers: ReadonlyMap<Trigger, TriggerBinding>,
): void {
  const owners = new Map<number, RigidBody>();
  for (const [body, binding] of bodies)
    for (const handle of binding.sources.keys()) owners.set(handle, body);

  const overlaps: Map<Trigger, Set<RigidBody>> = new Map();
  for (const [trigger, binding] of triggers) {
    const touching = new Set<RigidBody>();
    const parent = ancestorBody(trigger);
    for (const collider of binding.colliders)
      backend.intersectionPairsWith(collider, (other) => {
        const body = owners.get(other.handle);
        if (body && body !== parent) touching.add(body);
      });
    overlaps.set(trigger, touching);
  }
  const contacts: Map<RigidBody, Set<RigidBody>> = new Map();
  for (const [body, binding] of bodies) {
    const touching = new Set<RigidBody>();
    for (const handle of binding.sources.keys()) {
      const collider = backend.getCollider(handle);
      if (!collider) throw new Error("Missing body collider");
      backend.contactPairsWith(collider, (other) => {
        const owner = owners.get(other.handle);
        if (!owner || owner === body) return;
        backend.contactPair(collider, other, (manifold) => {
          if (manifold.numSolverContacts() > 0) touching.add(owner);
        });
      });
    }
    contacts.set(body, touching);
  }
  interactions.replace(overlaps, contacts);
}
