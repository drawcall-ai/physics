import { RigidBody, type CollisionGroups, type Joint } from "@drawcall/physics";
import { resolveCollisionGroups } from "@drawcall/physics/backend";
import { name } from "./markup.js";
import { matches, type Geometry } from "./shapes.js";

interface Mask {
  groups: CollisionGroups;
  contype: number;
  conaffinity: number;
  /** Groups this one collides with that no bit pairs it with yet. */
  open: Set<Mask>;
}

/**
 * Contact filtering through MuJoCo's own broadphase.
 *
 * MuJoCo collides two geometries when (a.contype & b.conaffinity) or (b.contype &
 * a.conaffinity) is set; collision groups collide only when each admits the other. So each
 * distinct group gets fresh bits: groups that all collide with one another and with
 * themselves share one bit in both masks, and each remaining colliding pair of groups gets a
 * bit from one to the other. Friction and restitution combine by MuJoCo's own rules.
 */
export function contacts(bodies: readonly RigidBody[], fixedDelta: number) {
  const key = (groups: CollisionGroups) =>
    `${groups.membership}/${groups.filter}`;
  const distinct = new Map<string, CollisionGroups>();
  for (const body of bodies)
    for (const collider of body.getColliders()) {
      const groups = resolveCollisionGroups(collider, body);
      distinct.set(key(groups), groups);
    }
  const masks = new Map(
    groupMasks([...distinct.values()]).map((mask) => [key(mask.groups), mask]),
  );
  const stiffness = Math.max(0.004, fixedDelta * 2);
  return (geometry: Geometry): string => {
    if (!(geometry.owner instanceof RigidBody))
      return 'contype="0" conaffinity="0"';
    const mask = masks.get(key(geometry.groups));
    if (!mask) throw new Error("Missing MuJoCo contact mask");
    const restitution = Math.min(geometry.restitution, 0.9999);
    const damping =
      restitution === 0
        ? 1
        : -Math.log(restitution) /
          Math.sqrt(Math.PI ** 2 + Math.log(restitution) ** 2);
    return `contype="${mask.contype}" conaffinity="${mask.conaffinity}" condim="3" friction="${geometry.friction} 0 0" solref="${stiffness} ${damping}"`;
  };
}

/** contype and conaffinity for each group, colliding exactly as `matches` says. */
function groupMasks(groups: CollisionGroups[]): Mask[] {
  const masks = groups.map((groups): Mask => ({
    groups,
    contype: 0,
    conaffinity: 0,
    open: new Set(),
  }));
  for (const a of masks)
    for (const b of masks) if (matches(a.groups, b.groups)) a.open.add(b);
  let bits = 0;
  const bit = () => {
    if (bits === 32)
      throw new Error(
        "Collision groups need more than MuJoCo's 32 contact bits",
      );
    return 1 << bits++;
  };
  const self = (mask: Mask) => matches(mask.groups, mask.groups);
  for (const a of masks)
    for (const b of masks) {
      if (!self(a) || !self(b) || !a.open.has(b)) continue;
      const shared = [...new Set([a, b])];
      for (const mask of masks)
        if (
          self(mask) &&
          !shared.includes(mask) &&
          shared.every((other) => matches(other.groups, mask.groups))
        )
          shared.push(mask);
      const value = bit();
      for (const mask of shared) {
        mask.contype |= value;
        mask.conaffinity |= value;
        for (const other of shared) mask.open.delete(other);
      }
    }
  // What is left pairs two groups of which at least one does not collide with itself.
  for (const a of masks)
    for (const b of [...a.open]) {
      const value = bit();
      a.contype |= value;
      b.conaffinity |= value;
      a.open.delete(b);
      b.open.delete(a);
    }
  return masks;
}

/**
 * Body pairs that never touch: bodies joined without collideConnected, and kinematic bodies
 * with any body that does not move freely. MuJoCo itself skips bodies welded together, which
 * includes every pair of static bodies.
 */
export function exclusions(
  bodies: readonly RigidBody[],
  joints: Iterable<Joint>,
): string {
  const pairs = new Map<string, string>();
  const exclude = (a: RigidBody, b: RigidBody) => {
    const key = [name(a), name(b)].sort().join(" ");
    pairs.set(key, `<exclude body1="${name(a)}" body2="${name(b)}"/>`);
  };
  for (const joint of joints) {
    const { body0, body1 } = joint.options;
    if (joint.enabled && !joint.collideConnected && body0)
      exclude(body0, body1);
  }
  const idle = bodies.filter((body) => body.bodyType !== "dynamic");
  for (const body of idle)
    if (body.bodyType === "kinematic")
      for (const other of idle) if (other !== body) exclude(body, other);
  return [...pairs.values()].join("");
}
