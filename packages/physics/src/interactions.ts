import { RigidBody } from "./body.js";
import { Trigger } from "./trigger.js";
import { cleanup } from "./cleanup.js";
import type { Vector3 } from "three";
type Pairs<T> = Map<T, Set<RigidBody>>;

/** One contact point of a body in the last step. */
export interface Contact {
  otherBody: RigidBody;
  /** World coordinates. */
  point: Vector3;
  /** Unit vector, world coordinates, along which the other body pushes this one. */
  normal: Vector3;
  /** Normal force, N; zero where the engine applied none or exposes none. */
  force: number;
}
/** Each body's contact points in the last step. */
export type Contacts = Map<RigidBody, Contact[]>;

export function addContact(
  contacts: Contacts,
  body: RigidBody,
  contact: Contact,
): void {
  const list = contacts.get(body) ?? [];
  list.push(contact);
  contacts.set(body, list);
}

/** Reconcile complete samples, so compound-shape handoffs and rebuilds stay silent. */
export class Interactions {
  private overlaps: Pairs<Trigger> = new Map();
  private contacts: Contacts = new Map();
  private events: (() => void)[] = [];
  private delivering = false;

  bodies(trigger: Trigger): RigidBody[] {
    return [...(this.overlaps.get(trigger) ?? [])];
  }
  /** Copies, so callers cannot change what later calls return. */
  contactsOf(body: RigidBody): Contact[] {
    return (this.contacts.get(body) ?? []).map((contact) => ({
      ...contact,
      point: contact.point.clone(),
      normal: contact.normal.clone(),
    }));
  }
  remove(object: RigidBody | Trigger): void {
    const overlaps = without(this.overlaps, object);
    const contacts: Contacts = new Map();
    for (const [body, list] of this.contacts) {
      if (body === object) continue;
      contacts.set(
        body,
        list.filter(({ otherBody }) => otherBody !== object),
      );
    }
    this.replace(overlaps, contacts);
  }
  clear(): void {
    this.overlaps.clear();
    this.contacts.clear();
    this.events = [];
  }
  /** Delivers every queued event, including those listeners queue, even when some throw. */
  dispatch(): void {
    if (this.delivering) return;
    this.delivering = true;
    try {
      cleanup(this.drain(), "Physics event listeners failed");
    } finally {
      this.delivering = false;
    }
  }
  private *drain(): Generator<() => void> {
    while (this.events.length) {
      const batch = this.events;
      this.events = [];
      yield* batch;
    }
  }
  replace(overlaps: Pairs<Trigger>, contacts: Contacts): void {
    transitions(this.overlaps, overlaps, (trigger, body, entered) => {
      this.events.push(() =>
        trigger.dispatchEvent({ type: entered ? "enter" : "exit", body }),
      );
    });
    transitions(
      partners(this.contacts),
      partners(contacts),
      (body, otherBody, entered) => {
        this.events.push(() =>
          body.dispatchEvent({
            type: entered ? "contactbegin" : "contactend",
            otherBody,
          }),
        );
      },
    );
    this.overlaps = overlaps;
    this.contacts = contacts;
  }
}

function partners(contacts: Contacts): Pairs<RigidBody> {
  const pairs: Pairs<RigidBody> = new Map();
  for (const [body, list] of contacts)
    pairs.set(body, new Set(list.map(({ otherBody }) => otherBody)));
  return pairs;
}

function without(
  pairs: Pairs<Trigger>,
  removed: RigidBody | Trigger,
): Pairs<Trigger> {
  const result: Pairs<Trigger> = new Map();
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
