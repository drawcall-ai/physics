import type { MainModule } from "@mujoco/mujoco";
import { RigidBody, Trigger, ancestorBody } from "@drawcall/physics";
import { addContact, type Contacts } from "@drawcall/physics/backend";
import type { Simulation } from "./model/compile.js";
import { matches } from "./model/shapes.js";
import { at, vector } from "./heap.js";

/** The bodies each trigger overlaps in the current state. */
export function sampleOverlaps(
  api: MainModule,
  sim: Simulation,
): Map<Trigger, Set<RigidBody>> {
  const overlaps = new Map<Trigger, Set<RigidBody>>();
  const buffer = new api.DoubleBuffer(6);
  try {
    for (const [a, geom] of sim.geometries) {
      if (!(geom.owner instanceof Trigger)) continue;
      const bodies = overlaps.get(geom.owner) ?? new Set<RigidBody>();
      overlaps.set(geom.owner, bodies);
      for (const [b, other] of sim.geometries) {
        if (
          !(other.owner instanceof RigidBody) ||
          other.owner === ancestorBody(geom.owner) ||
          !matches(geom.groups, other.groups)
        )
          continue;
        if (api.mj_geomDistance(sim.model, sim.data, a, b, 0.001, buffer) <= 0)
          bodies.add(other.owner);
      }
    }
  } finally {
    buffer.delete();
  }
  return overlaps;
}

/**
 * Each body's contact points with the normal force the last solve gave them. Call it between
 * mj_step2 and the next mj_step1, which detects new contacts while the forces stay the old ones.
 */
export function sampleContacts(api: MainModule, sim: Simulation): Contacts {
  const contacts: Contacts = new Map();
  const values = sim.data.contact;
  const force = new api.DoubleBuffer(6);
  try {
    for (let i = 0; i < values.size(); i++) {
      const contact = values.get(i);
      if (!contact) throw new Error("Missing MuJoCo contact");
      try {
        // Contacts without a constraint row (beyond the gap, or excluded) have no force.
        if (contact.efc_address < 0) continue;
        const first = sim.geometries.get(contact.geom1),
          second = sim.geometries.get(contact.geom2);
        if (!first || !second)
          throw new Error("Contact references an unknown MuJoCo geometry");
        const a = first.owner,
          b = second.owner;
        if (!(a instanceof RigidBody) || !(b instanceof RigidBody) || a === b)
          continue;
        api.mj_contactForce(sim.model, sim.data, i, force);
        const point = vector(contact.pos, 0);
        // The frame's first row is the normal from geom1 to geom2, along which b is pushed.
        const normal = vector(contact.frame, 0);
        const magnitude = at(force.GetView(), 0);
        addContact(contacts, b, {
          otherBody: a,
          point,
          normal,
          force: magnitude,
        });
        addContact(contacts, a, {
          otherBody: b,
          point: point.clone(),
          normal: normal.clone().negate(),
          force: magnitude,
        });
      } finally {
        contact.delete();
      }
    }
  } finally {
    force.delete();
    values.delete();
  }
  return contacts;
}
