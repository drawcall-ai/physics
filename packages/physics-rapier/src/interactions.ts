import type * as Rapier from "@dimforge/rapier3d-compat";
import { ancestorBody, type RigidBody, type Trigger } from "@drawcall/physics";
import type { Interactions } from "@drawcall/physics/backend";
import type { Owner } from "./query.js";

type Pairs<T> = Map<T, Set<RigidBody>>;

/** Samples the trigger overlaps and body contacts of the last step. */
export function sampleInteractions(
  interactions: Interactions,
  simulation: Rapier.World,
  owners: ReadonlyMap<number, Owner>,
): void {
  const overlaps: Pairs<Trigger> = new Map();
  const contacts: Pairs<RigidBody> = new Map();
  for (const [handle, owner] of owners) {
    const collider = simulation.getCollider(handle);
    if (owner.kind === "trigger") {
      const parent = ancestorBody(owner.trigger);
      simulation.intersectionPairsWith(collider, (other) => {
        const hit = owners.get(other.handle);
        if (hit?.kind === "body" && hit.body !== parent)
          pair(overlaps, owner.trigger, hit.body);
      });
      continue;
    }
    simulation.contactPairsWith(collider, (other) => {
      const hit = owners.get(other.handle);
      if (hit?.kind !== "body" || hit.body === owner.body) return;
      simulation.contactPair(collider, other, (manifold) => {
        if (manifold.numSolverContacts() > 0)
          pair(contacts, owner.body, hit.body);
      });
    });
  }
  interactions.replace(overlaps, contacts);
}

function pair<T>(pairs: Pairs<T>, owner: T, body: RigidBody): void {
  pairs.set(owner, (pairs.get(owner) ?? new Set()).add(body));
}
