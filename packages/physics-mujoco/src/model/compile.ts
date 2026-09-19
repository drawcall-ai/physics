import type { MainModule, MjModel, MjData } from "@mujoco/mujoco";
import {
  cleanup,
  rollback,
  RigidBody,
  Trigger,
  DistanceJoint,
  SphericalJoint,
  type Joint,
  type JointReading,
} from "@drawcall/physics";
import { Quaternion } from "three";
import type { Geometry } from "./shapes.js";
import type { JointRecord, Coordinate } from "./joints.js";
import { modelXml, type ModelOptions } from "./xml.js";
import { at, array, name, rotation } from "../values.js";
export type { ModelOptions } from "./xml.js";

export interface Compiled {
  model: MjModel;
  data: MjData;
  bodies: Map<RigidBody, number>;
  targets: Map<RigidBody, number>;
  triggers: Map<Trigger, number>;
  geometries: Map<number, Geometry>;
  coordinates: Map<number, Coordinate & { actuator: number; dof: number }>;
  roots: Set<RigidBody>;
  free(): void;
}
export function compile(
  api: MainModule,
  bodies: ReadonlySet<RigidBody>,
  joints: ReadonlyMap<Joint, JointRecord>,
  triggers: ReadonlySet<Trigger>,
  options: ModelOptions,
  read: (joint: Joint) => JointReading,
): Compiled {
  const { xml, geometries, coordinates, roots } = modelXml(
    bodies,
    joints,
    triggers,
    options,
    read,
  );
  const model = api.MjModel.from_xml_string(xml);
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
      [...bodies].map((body) => [
        body,
        id(api.mjtObj.mjOBJ_BODY.value, name(body)),
      ]),
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
      targets: new Map(
        [...bodies]
          .filter((body) => body.bodyType === "kinematic")
          .map((body) => [
            body,
            id(api.mjtObj.mjOBJ_BODY.value, `${name(body)}target`),
          ]),
      ),
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
