import {
  RigidBody,
  Trigger,
  DistanceJoint,
  splitTransform,
  treeJoint,
  type Joint,
  type JointReading,
} from "@drawcall/physics";
import { shapes, matches, type Geometry } from "./shapes.js";
import {
  jointXml,
  driveOf,
  type JointRecord,
  type Coordinate,
} from "./joints.js";
import { name, placement } from "../values.js";

import type { Meshes } from "./meshes.js";

export interface ModelOptions {
  meshes?: Meshes;
  fixedDelta: number;
  gravity: readonly number[];
  solverIterations: number;
  frictionImpedanceRatio: number;
  frictionCone: "pyramidal" | "elliptic";
}
export function modelXml(
  bodies: ReadonlySet<RigidBody>,
  joints: ReadonlyMap<Joint, JointRecord>,
  triggers: ReadonlySet<Trigger>,
  options: ModelOptions,
  read: (joint: Joint) => JointReading,
) {
  const parents = parentsOf(bodies, joints);
  const assets: string[] = [],
    geometries: Geometry[] = [],
    coordinates: Coordinate[] = [];
  const { sites, tendons, equalities } = distanceXml(joints);
  function contents(owner: RigidBody | Trigger): string {
    const result = shapes(owner, options.meshes);
    assets.push(...result.assets);
    geometries.push(...result.geometries);
    return result.xml;
  }
  function bodyXml(body: RigidBody): string {
    const parent = parents.get(body);
    const joint = parent?.joint;
    const record = parent?.record;
    const pose = record
      ? record.frames[0].clone().multiply(record.frames[1].clone().invert())
      : splitTransform(body.matrixWorld).pose;
    let connection =
      body.bodyType !== "static" && !joint
        ? `<freejoint name="${name(body)}free"/>`
        : "";
    if (joint && record)
      connection = jointXml(joint, record, read(joint), coordinates);
    const mass = body.options;
    const geometry = contents(body);
    const inertial = mass.centerOfMass
      ? `<inertial pos="${mass.centerOfMass.join(" ")}" mass="${mass.mass}" diaginertia="${mass.diagonalInertia.join(" ")}" quat="${[mass.principalAxes?.[3] ?? 1, mass.principalAxes?.[0] ?? 0, mass.principalAxes?.[1] ?? 0, mass.principalAxes?.[2] ?? 0].join(" ")}"/>`
      : body.bodyType === "kinematic" && !geometry
        ? `<inertial pos="0 0 0" mass="${mass.mass ?? 1}" diaginertia="1 1 1"/>`
        : "";
    const children = [...parents]
      .filter(([, edge]) => edge.joint.options.body0 === body)
      .map(([child]) => bodyXml(child))
      .join("");
    return `<body name="${name(body)}" ${placement(pose)} gravcomp="${body.bodyType === "kinematic" ? 1 : 1 - body.gravityScale}">${connection}${inertial}${geometry}${(sites.get(body) ?? []).join("")}${children}</body>`;
  }
  const roots = new Set([...bodies].filter((body) => !parents.has(body)));
  const xmlBodies =
    [...roots].map(bodyXml).join("") +
    [...parents]
      .filter(([, parent]) => !parent.joint.options.body0)
      .map(([body]) => bodyXml(body))
      .join("");
  const xmlTargets = [...bodies]
    .filter((body) => body.bodyType === "kinematic")
    .map((body) => {
      // A stiff native weld supplies contact velocity without teleporting collision geometry.
      // MuJoCo clamps positive solref time constants to at least two timesteps.
      equalities.push(
        `<weld body1="${name(body)}" body2="${name(body)}target" relpose="0 0 0 1 0 0 0" solref="${2 * options.fixedDelta} 1" solimp="0.999 0.999 0.001"/>`,
      );
      return `<body name="${name(body)}target" mocap="true" ${placement(splitTransform(body.matrixWorld).pose)}/>`;
    })
    .join("");
  const xmlTriggers = [...triggers]
    .map(
      (trigger) =>
        `<body name="${name(trigger)}" mocap="true" ${placement(splitTransform(trigger.matrixWorld).pose)}>${contents(trigger)}</body>`,
    )
    .join("");
  const pairs = contactXml(geometries, joints, options.fixedDelta);
  const actuators = coordinates
    .map((c) => {
      const max = driveOf(c)?.options.maxForce;
      return `<general name="${c.name}drive" joint="${c.name}" gainprm="1" biastype="affine" biasprm="0 0 0" ${max === undefined ? 'forcelimited="false"' : `forcelimited="true" forcerange="${-Math.max(max, 1e-30)} ${Math.max(max, 1e-30)}"`}/>`;
    })
    .join("");
  // Include position stiffness in the implicit solve; implicitfast lets stiff servos oscillate.
  const xml = `<mujoco><compiler angle="radian" fusestatic="false"/><option timestep="${options.fixedDelta}" gravity="${options.gravity.join(" ")}" iterations="${options.solverIterations}" cone="${options.frictionCone}" impratio="${options.frictionImpedanceRatio}" integrator="discrete"><flag filterparent="disable"/></option><asset>${assets.join("")}</asset><worldbody>${xmlBodies}${xmlTargets}${xmlTriggers}${(sites.get(null) ?? []).join("")}</worldbody><contact>${pairs.join("")}</contact><tendon>${tendons.join("")}</tendon><equality>${equalities.join("")}</equality><actuator>${actuators}</actuator></mujoco>`;
  return { xml, geometries, coordinates, roots };
}

function parentsOf(
  bodies: ReadonlySet<RigidBody>,
  joints: ReadonlyMap<Joint, JointRecord>,
) {
  const parents = new Map<RigidBody, { joint: Joint; record: JointRecord }>();
  for (const [joint, record] of joints) {
    if (!treeJoint(joint)) continue;
    const { body1 } = joint.options;
    if (body1.bodyType !== "dynamic")
      throw new Error("MuJoCo joint body1 must be dynamic");
    if (parents.has(body1))
      throw new Error(
        "MuJoCo constrained joints must form a tree; a body cannot have two parent joints",
      );
    parents.set(body1, { joint, record });
  }
  for (const body of bodies) {
    const visited = new Set<RigidBody>();
    let node: RigidBody | null = body;
    while (node) {
      if (visited.has(node))
        throw new Error("MuJoCo constrained joints cannot form a cycle");
      visited.add(node);
      node = parents.get(node)?.joint.options.body0 ?? null;
    }
  }
  return parents;
}

function distanceXml(joints: ReadonlyMap<Joint, JointRecord>) {
  const tendons: string[] = [],
    equalities: string[] = [];
  const sites = new Map<RigidBody | null, string[]>();
  for (const [joint, record] of joints) {
    if (!(joint instanceof DistanceJoint) || !joint.enabled) continue;
    for (const index of [0, 1] as const) {
      const owner = index === 0 ? joint.options.body0 : joint.options.body1;
      const list = sites.get(owner) ?? [];
      list.push(
        `<site name="${name(joint)}s${index}" ${placement(record.frames[index])} size="0.001"/>`,
      );
      sites.set(owner, list);
    }
    const [min, max] = joint.limits;
    const limited = Number.isFinite(max) && min !== max;
    tendons.push(
      `<spatial name="${name(joint)}" ${limited ? `limited="true" range="${min} ${max}"` : 'limited="false"'}><site site="${name(joint)}s0"/><site site="${name(joint)}s1"/></spatial>`,
    );
    if (min === max)
      equalities.push(
        `<tendon name="${name(joint)}" tendon1="${name(joint)}" polycoef="0 0 0 0 0"/>`,
      );
    if (!Number.isFinite(max) && min > 0)
      throw new Error(
        "MuJoCo distance joints with an infinite maximum require a zero minimum",
      );
  }
  return { sites, tendons, equalities };
}

function contactXml(
  geometries: Geometry[],
  joints: ReadonlyMap<Joint, JointRecord>,
  fixedDelta: number,
) {
  const pairs: string[] = [];
  for (const [i, a] of geometries.entries()) {
    if (!(a.owner instanceof RigidBody)) continue;
    for (const b of geometries.slice(i + 1)) {
      if (
        !(b.owner instanceof RigidBody) ||
        a.owner === b.owner ||
        !matches(a.groups, b.groups)
      )
        continue;
      if (a.owner.bodyType !== "dynamic" && b.owner.bodyType !== "dynamic")
        continue;
      const ownerA = a.owner,
        ownerB = b.owner;
      if (
        [...joints.keys()].some(
          (j) =>
            j.enabled &&
            !j.collideConnected &&
            j.connects(ownerA) &&
            j.connects(ownerB),
        )
      )
        continue;
      const friction = Math.sqrt(a.friction * b.friction);
      const restitution = Math.max(a.restitution, b.restitution);
      const damping =
        restitution === 0
          ? 1
          : -Math.log(Math.min(restitution, 0.9999)) /
            Math.sqrt(
              Math.PI ** 2 + Math.log(Math.min(restitution, 0.9999)) ** 2,
            );
      pairs.push(
        `<pair geom1="${a.name}" geom2="${b.name}" condim="3" friction="${friction} ${friction} 0 0 0" solref="${Math.max(0.004, fixedDelta * 2)} ${damping}"/>`,
      );
    }
  }
  return pairs;
}
