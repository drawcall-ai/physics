import type * as Rapier from "@dimforge/rapier3d-compat";
import { RigidBody, Trigger, ancestorBody } from "@drawcall/physics";
import type { BodyBinding } from "./body.js";
import type { TriggerBinding } from "./triggers.js";

type Pairs<T> = Map<T, Set<RigidBody>>;

/** Reconcile complete samples, so compound-shape handoffs and rebuilds stay silent. */
export class Interactions {
  private overlaps: Pairs<Trigger> = new Map();
  private contacts: Pairs<RigidBody> = new Map();
  private events: (() => void)[] = [];
  private delivering = false;

  get dispatching(): boolean {
    return this.delivering;
  }
  bodies(trigger: Trigger): RigidBody[] {
    return [...(this.overlaps.get(trigger) ?? [])];
  }
  sample(
    backend: Rapier.World,
    bodies: ReadonlyMap<RigidBody, BodyBinding>,
    triggers: ReadonlyMap<Trigger, TriggerBinding>,
  ): void {
    const owners = new Map<number, RigidBody>();
    for (const [body, binding] of bodies)
      for (const handle of binding.sources.keys()) owners.set(handle, body);

    const overlaps: Pairs<Trigger> = new Map();
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
    const contacts: Pairs<RigidBody> = new Map();
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
    this.replace(overlaps, contacts);
  }
  remove(object: RigidBody | Trigger): void {
    const overlaps = without(this.overlaps, object);
    const contacts = without(this.contacts, object);
    this.replace(overlaps, contacts);
  }
  clear(): void {
    this.overlaps.clear();
    this.contacts.clear();
    this.events = [];
  }
  dispatch(): void {
    if (this.delivering) return;
    this.delivering = true;
    try {
      for (let i = 0; i < this.events.length; i++) this.events[i]?.();
    } finally {
      this.events = [];
      this.delivering = false;
    }
  }
  private replace(overlaps: Pairs<Trigger>, contacts: Pairs<RigidBody>): void {
    transitions(this.overlaps, overlaps, (trigger, body, entered) => {
      this.events.push(() => {
        if (!trigger.disposed && (!entered || !body.disposed))
          trigger.dispatchEvent({ type: entered ? "enter" : "exit", body });
      });
    });
    transitions(this.contacts, contacts, (body, otherBody, entered) => {
      this.events.push(() => {
        if (!body.disposed && (!entered || !otherBody.disposed))
          body.dispatchEvent({
            type: entered ? "contactbegin" : "contactend",
            otherBody,
          });
      });
    });
    this.overlaps = overlaps;
    this.contacts = contacts;
  }
}

function without<T extends RigidBody | Trigger>(
  pairs: Pairs<T>,
  removed: RigidBody | Trigger,
): Pairs<T> {
  const result: Pairs<T> = new Map();
  for (const [owner, bodies] of pairs) {
    if (owner === removed) continue;
    result.set(owner, new Set([...bodies].filter((body) => body !== removed)));
  }
  return result;
}

function transitions<T>(
  previous: Pairs<T>,
  next: Pairs<T>,
  emit: (owner: T, body: RigidBody, entered: boolean) => void,
): void {
  for (const [owner, bodies] of previous)
    for (const body of bodies)
      if (!next.get(owner)?.has(body)) emit(owner, body, false);
  for (const [owner, bodies] of next)
    for (const body of bodies)
      if (!previous.get(owner)?.has(body)) emit(owner, body, true);
}
