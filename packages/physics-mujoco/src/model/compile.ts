import type { MainModule, MjModel, MjData } from "@mujoco/mujoco";
import {
  RigidBody,
  Trigger,
  DistanceJoint,
  SphericalJoint,
  type Joint,
  type SoftBody,
  type SoftAttachment,
} from "@drawcall/physics";
import {
  cleanup,
  rollback,
  type JointBinding,
} from "@drawcall/physics/backend";
import { Quaternion } from "three";
import type { Geometry } from "./shapes.js";
import type { Coordinate } from "./joints.js";
import { modelXml, type ModelOptions } from "./xml.js";
import { name } from "./markup.js";
import type { SoftMesh } from "../soft/mesh.js";
import {
  correctMembraneStiffness,
  nodeName,
  type Attachment,
} from "../soft/model.js";
import type { SoftState } from "../soft/state.js";
import { jointName, type ReducedMesh } from "../reduced/model.js";
import type { ReducedState } from "../reduced/state.js";
import { at, array, rotation } from "../heap.js";
export type { ModelOptions } from "./xml.js";

/** A compiled MuJoCo model, its simulation data, and where each physics object landed in them. */
export interface Simulation {
  model: MjModel;
  data: MjData;
  bodies: Map<RigidBody, number>;
  soft: Map<SoftBody, SoftState>;
  reduced: Map<SoftBody, ReducedState>;
  targets: Map<RigidBody, number>;
  triggers: Map<Trigger, number>;
  geometries: Map<number, Geometry>;
  coordinates: Map<number, Coordinate & { actuator: number; dof: number }>;
  free(): void;
}
export function compile(
  api: MainModule,
  bodies: readonly RigidBody[],
  joints: ReadonlyMap<Joint, JointBinding>,
  triggers: readonly Trigger[],
  options: ModelOptions,
  soft: ReadonlyMap<SoftBody, SoftMesh> = new Map(),
  attachments: Iterable<Attachment> = [],
  reduced: ReadonlyMap<SoftBody, ReducedMesh> = new Map(),
  reducedAttachments: readonly SoftAttachment[] = [],
): Simulation {
  // The WASM distance query only accepts geoms; it cannot report flex-trigger overlaps.
  if ((soft.size || reduced.size) && triggers.length)
    throw new Error(
      "MuJoCo soft bodies cannot share a world with triggers: flex overlap queries are unavailable",
    );
  const { xml, geometries, coordinates } = modelXml(
    bodies,
    joints,
    triggers,
    options,
    soft,
    attachments,
    reduced,
    reducedAttachments,
  );
  const spec = api.parseXMLString(xml);
  const release = [() => spec.delete()];
  let model: MjModel;
  try {
    for (const body of soft.keys()) {
      if (body.material.elasticity.model !== "neoHookean") continue;
      // MuJoCo 3.16 exposes stable Neo-Hookean through mjSpec only, not MJCF.
      const element = api.mjs_findElement(
        spec,
        api.mjtObj.mjOBJ_FLEX,
        name(body),
      );
      if (!element) throw new Error(`Missing MuJoCo flex: ${name(body)}`);
      release.unshift(() => element.delete());
      const flex = api.mjs_asFlex(element);
      if (!flex) throw new Error(`Invalid MuJoCo flex: ${name(body)}`);
      release.unshift(() => flex.delete());
      flex.elastic3d = 1;
    }
    model = api.mj_compile(spec);
  } catch (error) {
    rollback(error, release, "MuJoCo specification compilation failed");
  }
  try {
    cleanup(release, "MuJoCo specification disposal failed");
  } catch (error) {
    rollback(
      error,
      [() => model.delete()],
      "MuJoCo specification disposal failed",
    );
  }
  let data: MjData;
  try {
    data = new api.MjData(model);
  } catch (error) {
    rollback(error, [() => model.delete()], "MuJoCo data creation failed");
  }
  try {
    const id = (kind: number, key: string) => {
      const value = api.mj_name2id(model, kind, key);
      if (value < 0) throw new Error(`Missing compiled MuJoCo object: ${key}`);
      return value;
    };
    const bodyIds = new Map(
      bodies.map((body) => [body, id(api.mjtObj.mjOBJ_BODY.value, name(body))]),
    );
    for (const [body, index] of bodyIds) {
      if (
        body.options.mass === undefined ||
        body.options.centerOfMass ||
        body.bodyType === "static"
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
    correctMembraneStiffness(model);
    api.mj_setConst(model, data);
    api.mj_resetData(model, data);
    for (const coordinate of coordinates) {
      const joint = id(api.mjtObj.mjOBJ_JOINT.value, coordinate.name);
      array(data.qpos)[at(model.jnt_qposadr, joint)] = coordinate.position;
    }
    for (const [joint, binding] of joints) {
      if (!joint.enabled) continue;
      if (joint instanceof SphericalJoint) {
        const frame = new Quaternion().setFromRotationMatrix(binding.frames[1]);
        const value = frame
          .clone()
          .multiply(binding.pose().rotation)
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
      soft: new Map(
        [...soft].map(([body, mesh]) => [
          body,
          {
            mesh,
            id: id(api.mjtObj.mjOBJ_FLEX.value, name(body)),
            nodes: Array.from(mesh.volumes, (_, node) =>
              id(api.mjtObj.mjOBJ_BODY.value, nodeName(mesh, node)),
            ),
          },
        ]),
      ),
      reduced: new Map(
        [...reduced].map(([body, mesh]) => [
          body,
          {
            mesh,
            id: id(api.mjtObj.mjOBJ_FLEX.value, name(body)),
            joints: Array.from({ length: mesh.model.dofs + 3 }, (_, i) =>
              id(api.mjtObj.mjOBJ_JOINT.value, jointName(body, i)),
            ),
            springs: Array.from({ length: mesh.model.dofs }, (_, i) =>
              id(api.mjtObj.mjOBJ_TENDON.value, `${name(body)}k${i}`),
            ),
          },
        ]),
      ),
      targets: new Map(
        bodies
          .filter((body) => body.bodyType === "kinematic")
          .map((body) => [
            body,
            id(api.mjtObj.mjOBJ_BODY.value, `${name(body)}target`),
          ]),
      ),
      triggers: new Map(
        triggers.map((trigger) => [
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
          {
            ...coordinate,
            actuator: id(
              api.mjtObj.mjOBJ_ACTUATOR.value,
              `${coordinate.name}drive`,
            ),
            dof: at(
              model.jnt_dofadr,
              id(api.mjtObj.mjOBJ_JOINT.value, coordinate.name),
            ),
          },
        ]),
      ),
      free() {
        cleanup(
          [() => data.delete(), () => model.delete()],
          "MuJoCo model disposal failed",
        );
      },
    };
  } catch (error) {
    rollback(
      error,
      [() => data.delete(), () => model.delete()],
      "MuJoCo model compilation failed",
    );
  }
}
