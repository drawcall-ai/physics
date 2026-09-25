import {
  RigidBody,
  resolveCollisionGroups,
  type CollisionGroups,
  type Joint,
} from "@drawcall/physics";
import { name } from "../values.js";
import { matches, type Geometry } from "./shapes.js";

/**
 * Contact filtering through MuJoCo's own broadphase.
 *
 * MuJoCo collides two geometries when (a.contype & b.conaffinity) or (b.contype &
 * a.conaffinity) is set; collision groups collide only when each admits the other. So each
 * distinct group gets fresh bits: groups that all collide with one another and with
 * themselves share one bit in both masks, and each remaining colliding pair of groups gets a
 * bit from one to the other. Friction and restitution combine by MuJoCo's own rules.
 */
export function contacts(bodies: ReadonlySet<RigidBody>, fixedDelta: number) {
  const key = (groups: CollisionGroups) =>
    `${groups.membership}/${groups.filter}`;
  const distinct = new Map<string, CollisionGroups>();
  for (const body of bodies)
    for (const collider of body.getColliders()) {
      const groups = resolveCollisionGroups(collider, body);
      distinct.set(key(groups), groups);
    }
  const groups = [...distinct.values()];
  const masks = new Map(
    groupMasks(groups).map((mask, i) => [key(groups[i]!), mask]),
  );
  const stiffness = Math.max(0.004, fixedDelta * 2);
  return (geometry: Geometry): string => {
    if (!(geometry.owner instanceof RigidBody))
      return 'contype="0" conaffinity="0"';
    const [contype, conaffinity] = masks.get(key(geometry.groups))!;
    const restitution = Math.min(geometry.restitution, 0.9999);
    const damping =
      restitution === 0
        ? 1
        : -Math.log(restitution) /
          Math.sqrt(Math.PI ** 2 + Math.log(restitution) ** 2);
    return `contype="${contype}" conaffinity="${conaffinity}" condim="3" friction="${geometry.friction} 0 0" solref="${stiffness} ${damping}"`;
  };
}

/** contype and conaffinity for each group, colliding exactly as `matches` says. */
function groupMasks(groups: CollisionGroups[]): [number, number][] {
  const masks = groups.map((): [number, number] => [0, 0]);
  const pair = (i: number, j: number) => `${Math.min(i, j)},${Math.max(i, j)}`;
  const open = new Set<string>();
  for (const [i, a] of groups.entries())
    for (const [j, b] of groups.entries())
      if (i <= j && matches(a, b)) open.add(pair(i, j));
  let bits = 0;
  const bit = () => {
    if (bits === 32)
      throw new Error(
        "Collision groups need more than MuJoCo's 32 contact bits",
      );
    return 1 << bits++;
  };
  const selfColliding = groups.map((g) => matches(g, g));
  for (const [i] of groups.entries())
    for (const [j] of groups.entries()) {
      if (!selfColliding[i] || !selfColliding[j] || !open.has(pair(i, j)))
        continue;
      const shared = [...new Set([i, j])];
      for (const [k, group] of groups.entries())
        if (
          selfColliding[k] &&
          !shared.includes(k) &&
          shared.every((m) => matches(groups[m]!, group))
        )
          shared.push(k);
      const b = bit();
      for (const m of shared) {
        masks[m]![0] |= b;
        masks[m]![1] |= b;
        for (const n of shared) open.delete(pair(m, n));
      }
    }
  // What is left pairs two groups of which at least one does not collide with itself.
  for (const key of open) {
    const [i, j] = key.split(",").map(Number) as [number, number];
    const b = bit();
    masks[i]![0] |= b;
    masks[j]![1] |= b;
  }
  return masks;
}

/**
 * Body pairs that never touch: bodies joined without collideConnected, and kinematic bodies
 * with any body that does not move freely. MuJoCo itself skips bodies welded together, which
 * includes every pair of static bodies.
 */
export function exclusions(
  bodies: ReadonlySet<RigidBody>,
  joints: Iterable<Joint>,
): string {
  const pairs = new Map<string, string>();
  const exclude = (a: RigidBody, b: RigidBody) => {
    const key = [name(a), name(b)].sort().join(" ");
    pairs.set(key, `<exclude body1="${name(a)}" body2="${name(b)}"/>`);
  };
  for (const joint of joints) {
    const { body0, body1 } = joint.options;
    if (joint.enabled && !joint.collideConnected && body0 && bodies.has(body0))
      exclude(body0, body1);
  }
  const idle = [...bodies].filter((body) => body.bodyType !== "dynamic");
  for (const body of idle)
    if (body.bodyType === "kinematic")
      for (const other of idle) if (other !== body) exclude(body, other);
  return [...pairs.values()].join("");
}
