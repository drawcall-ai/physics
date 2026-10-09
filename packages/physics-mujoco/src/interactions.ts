import type { MainModule } from "@mujoco/mujoco";
import { RigidBody, Trigger, ancestorBody } from "@drawcall/physics";
import type { Interactions } from "@drawcall/physics/backend";
import type { Simulation } from "./model/compile.js";
import { matches } from "./model/shapes.js";

/** Replaces the trigger overlaps and body contacts with those of the current state. */
export function sampleInteractions(
  api: MainModule,
  sim: Simulation,
  interactions: Interactions,
): void {
  const overlaps = new Map<Trigger, Set<RigidBody>>(),
    contacts = new Map<RigidBody, Set<RigidBody>>();
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
  const values = sim.data.contact;
  try {
    for (let i = 0; i < values.size(); i++) {
      const contact = values.get(i);
      if (!contact) throw new Error("Missing MuJoCo contact");
      try {
        if (contact.dist > 0) continue;
        const first = sim.geometries.get(contact.geom1),
          second = sim.geometries.get(contact.geom2);
        if (!first || !second)
          throw new Error("Contact references an unknown MuJoCo geometry");
        const a = first.owner,
          b = second.owner;
        if (!(a instanceof RigidBody) || !(b instanceof RigidBody) || a === b)
          continue;
        for (const [body, other] of [
          [a, b],
          [b, a],
        ] as const) {
          const set = contacts.get(body) ?? new Set<RigidBody>();
          set.add(other);
          contacts.set(body, set);
        }
      } finally {
        contact.delete();
      }
    }
  } finally {
    values.delete();
  }
  interactions.replace(overlaps, contacts);
}
