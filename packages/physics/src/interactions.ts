import { RigidBody } from "./body.js";
import { Trigger } from "./trigger.js";
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
  replace(overlaps: Pairs<Trigger>, contacts: Pairs<RigidBody>): void {
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
