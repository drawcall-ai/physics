import type { MainModule, MjModel, MjData } from "@mujoco/mujoco";
import {
  RigidBody,
  Trigger,
  DistanceJoint,
  SphericalJoint,
  splitTransform,
  type Joint,
  type PhysicsVelocity,
} from "@drawcall/physics";
import { Matrix4, Quaternion } from "three";
import { shapes, matches, type Geometry } from "./shapes.js";
import {
  jointXml,
  treeJoint,
  driveOf,
  type JointRecord,
  type Coordinate,
} from "./joints.js";
import { at, array, name, placement, rotation } from "./values.js";

export interface BodyRecord {
  initialPose: Matrix4;
  initialVelocity: PhysicsVelocity;
}
export interface Compiled {
  model: MjModel;
  data: MjData;
  bodies: Map<RigidBody, number>;
  triggers: Map<Trigger, number>;
  geometries: Map<number, Geometry>;
  coordinates: Map<number, Coordinate>;
  roots: Set<RigidBody>;
  free(): void;
}
export interface ModelOptions {
  fixedDelta: number;
  gravity: readonly number[];
  solverIterations: number;
}
export function compile(
  api: MainModule,
  bodies: ReadonlyMap<RigidBody, BodyRecord>,
  joints: ReadonlyMap<Joint, JointRecord>,
  triggers: ReadonlySet<Trigger>,
  options: ModelOptions,
  read: (joint: Joint) => import("@drawcall/physics").JointReading,
): Compiled {
  const parents = new Map<RigidBody, Joint>();
  for (const joint of joints.keys()) {
    if (!treeJoint(joint)) continue;
    const { body1 } = joint.options;
    if (body1.bodyType !== "dynamic")
      throw new Error("MuJoCo joint body1 must be dynamic");
    if (parents.has(body1))
      throw new Error(
        "MuJoCo constrained joints must form a tree; a body cannot have two parent joints",
      );
    parents.set(body1, joint);
  }
  for (const body of bodies.keys()) {
    const visited = new Set<RigidBody>();
    let node: RigidBody | null = body;
    while (node) {
      if (visited.has(node))
        throw new Error("MuJoCo constrained joints cannot form a cycle");
      visited.add(node);
      node = parents.get(node)?.options.body0 ?? null;
    }
  }
  const assets: string[] = [],
    geometries: Geometry[] = [],
    coordinates: Coordinate[] = [],
    tendons: string[] = [],
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
  function contents(owner: RigidBody | Trigger): string {
    const result = shapes(owner);
    assets.push(...result.assets);
    geometries.push(...result.geometries);
    return result.xml;
  }
  function bodyXml(body: RigidBody): string {
    const joint = parents.get(body);
    const record = joint && joints.get(joint);
    const pose = record
      ? record.frames[0].clone().multiply(record.frames[1].clone().invert())
      : splitTransform(body.matrixWorld).pose;
    let connection =
      body.bodyType === "dynamic" && !joint
        ? `<freejoint name="${name(body)}free"/>`
        : "";
    if (joint && record)
      connection = jointXml(joint, record, read(joint), coordinates);
    const mass = body.options;
    const inertial = mass.centerOfMass
      ? `<inertial pos="${mass.centerOfMass.join(" ")}" mass="${mass.mass}" diaginertia="${mass.diagonalInertia.join(" ")}" quat="${[mass.principalAxes?.[3] ?? 1, mass.principalAxes?.[0] ?? 0, mass.principalAxes?.[1] ?? 0, mass.principalAxes?.[2] ?? 0].join(" ")}"/>`
      : "";
    const children = [...parents]
      .filter(([, edge]) => edge.options.body0 === body)
      .map(([child]) => bodyXml(child))
      .join("");
    return `<body name="${name(body)}" ${placement(pose)} ${body.bodyType === "kinematic" ? 'mocap="true"' : ""} gravcomp="${1 - body.gravityScale}">${connection}${inertial}${contents(body)}${(sites.get(body) ?? []).join("")}${children}</body>`;
  }
  const roots = new Set(
    [...bodies.keys()].filter((body) => !parents.has(body)),
  );
  const xmlBodies =
    [...roots].map(bodyXml).join("") +
    [...parents]
      .filter(([, joint]) => !joint.options.body0)
      .map(([body]) => bodyXml(body))
      .join("");
  const xmlTriggers = [...triggers]
    .map(
      (trigger) =>
        `<body name="${name(trigger)}" mocap="true" ${placement(splitTransform(trigger.matrixWorld).pose)}>${contents(trigger)}</body>`,
    )
    .join("");
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
        `<pair geom1="${a.name}" geom2="${b.name}" condim="3" friction="${friction} ${friction} 0 0 0" solref="${Math.max(0.004, options.fixedDelta * 2)} ${damping}"/>`,
      );
    }
  }
  const actuators = coordinates
    .map((c) => {
      const max = driveOf(c)?.options.maxForce;
      return `<general name="${c.name}drive" joint="${c.name}" gainprm="1" biastype="affine" biasprm="0 0 0" ${max === undefined ? 'forcelimited="false"' : `forcelimited="true" forcerange="${-Math.max(max, 1e-30)} ${Math.max(max, 1e-30)}"`}/>`;
    })
    .join("");
  // Include position stiffness in the implicit solve; implicitfast lets stiff servos oscillate.
  const xml = `<mujoco><compiler angle="radian" fusestatic="false"/><option timestep="${options.fixedDelta}" gravity="${options.gravity.join(" ")}" iterations="${options.solverIterations}" integrator="discrete"><flag filterparent="disable"/></option><asset>${assets.join("")}</asset><worldbody>${xmlBodies}${xmlTriggers}${(sites.get(null) ?? []).join("")}</worldbody><contact>${pairs.join("")}</contact><tendon>${tendons.join("")}</tendon><equality>${equalities.join("")}</equality><actuator>${actuators}</actuator></mujoco>`;
  const model = api.MjModel.from_xml_string(xml);
  let data: MjData;
  try {
    data = new api.MjData(model);
  } catch (error) {
    model.delete();
    throw error;
  }
  try {
    const id = (kind: number, key: string) => {
      const value = api.mj_name2id(model, kind, key);
      if (value < 0) throw new Error(`Missing compiled MuJoCo object: ${key}`);
      return value;
    };
    const bodyIds = new Map(
      [...bodies.keys()].map((body) => [
        body,
        id(api.mjtObj.mjOBJ_BODY.value, name(body)),
      ]),
    );
    for (const [body, index] of bodyIds) {
      if (
        body.options.mass === undefined ||
        body.options.centerOfMass ||
        body.bodyType !== "dynamic"
      )
        continue;
      const inferred = at(model.body_mass, index);
      if (inferred <= 0)
        throw new Error("Dynamic body requires positive mass and inertia");
      const ratio = body.options.mass / inferred;
      array(model.body_mass)[index] = body.options.mass;
      for (let j = 0; j < 3; j++)
        array(model.body_inertia)[index * 3 + j] =
          at(model.body_inertia, index * 3 + j) * ratio;
    }
    api.mj_setConst(model, data);
    api.mj_resetData(model, data);
    for (const coordinate of coordinates) {
      const joint = id(api.mjtObj.mjOBJ_JOINT.value, coordinate.name);
      array(data.qpos)[at(model.jnt_qposadr, joint)] = coordinate.position;
    }
    for (const [joint, record] of joints) {
      if (!joint.enabled) continue;
      if (joint instanceof SphericalJoint) {
        const frame = new Quaternion().setFromRotationMatrix(record.frames[1]);
        const value = frame
          .clone()
          .multiply(read(joint).rotation)
          .multiply(frame.invert());
        const index = id(api.mjtObj.mjOBJ_JOINT.value, name(joint));
        array(data.qpos).set(rotation(value), at(model.jnt_qposadr, index));
      }
      if (
        joint instanceof DistanceJoint &&
        joint.limits[0] === joint.limits[1]
      ) {
        const index = id(api.mjtObj.mjOBJ_TENDON.value, name(joint));
        const equality = id(api.mjtObj.mjOBJ_EQUALITY.value, name(joint));
        array(model.eq_data)[equality * api.mjNEQDATA] =
          joint.limits[0] - at(model.tendon_length0, index);
      }
    }
    api.mj_forward(model, data);
    return {
      model,
      data,
      bodies: bodyIds,
      roots,
      triggers: new Map(
        [...triggers].map((trigger) => [
          trigger,
          id(api.mjtObj.mjOBJ_BODY.value, name(trigger)),
        ]),
      ),
      geometries: new Map(
        geometries.map((geom) => [
          id(api.mjtObj.mjOBJ_GEOM.value, geom.name),
          geom,
        ]),
      ),
      coordinates: new Map(
        coordinates.map((coordinate) => [
          id(api.mjtObj.mjOBJ_JOINT.value, coordinate.name),
          coordinate,
        ]),
      ),
      free() {
        data.delete();
        model.delete();
      },
    };
  } catch (error) {
    data.delete();
    model.delete();
    throw error;
  }
}
