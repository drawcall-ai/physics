import {
  SoftBody,
  SoftAttachment,
  splitTransform,
  type ReducedModel,
  type RigidBody,
} from "@drawcall/physics";
import { selectSoftNodes } from "@drawcall/physics/backend";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { Geometry } from "../model/shapes.js";
import { name, placement } from "../model/markup.js";
import { cholesky, serial, value } from "./matrix.js";

export interface ReducedMesh {
  body: SoftBody;
  model: ReducedModel;
  pose: Matrix4;
  mountRotation?: Quaternion;
  anchor?: Vector3;
  target?: RigidBody | null;
}

export function reducedMesh(body: SoftBody): ReducedMesh {
  body.validate();
  if (!body.model) throw new Error("Missing reduced FEM model");
  const { pose, scale } = splitTransform(body.matrixWorld);
  if (scale.distanceTo(new Vector3(1, 1, 1)) > 1e-6)
    throw new Error("Reduced FEM geometry must be baked at its physical scale");
  return { body, model: body.model, pose };
}

export function jointName(body: SoftBody, dof: number): string {
  return `${name(body)}q${dof}`;
}

export function reducedXml(
  meshes: Iterable<ReducedMesh>,
  attachments: readonly SoftAttachment[],
  contact: (geometry: Geometry) => string,
  timestep: number,
) {
  const bodies: string[] = [],
    flexes: string[] = [],
    tendons: string[] = [],
    equalities: string[] = [];
  const sites = new Map<RigidBody | null, string[]>();
  for (const mesh of meshes) {
    const { body, model, pose } = mesh;
    const matches = attachments.filter(
      (attachment) => attachment.options.body === body,
    );
    const mount = matches[0];
    if (matches.length !== 1 || !mount?.enabled)
      throw new Error(
        "Reduced FEM requires exactly one enabled attachment at its baked mount",
      );
    const selected = selectSoftNodes(mount, model.positions).sort(
      (a, b) => a - b,
    );
    const expected = [...model.mount].sort((a, b) => a - b);
    if (
      selected.length !== expected.length ||
      selected.some((node, i) => node !== expected[i])
    )
      throw new Error(
        "Reduced FEM attachment must match the baked mount region",
      );
    const target = mount.options.target;
    if (target?.bodyType === "dynamic")
      throw new Error(
        "MuJoCo reduced FEM currently supports world, static, or translating kinematic mounts",
      );
    target?.updateWorldMatrix(true, false);
    const rotation = target
      ? new Quaternion().setFromRotationMatrix(
          splitTransform(target.matrixWorld).pose,
        )
      : new Quaternion();
    // Kinematic desired rotations are checked by writeTarget. Their compliant
    // native bodies can acquire tiny angular errors under load before a rebuild.
    if (
      target?.bodyType !== "kinematic" &&
      mesh.mountRotation &&
      mesh.mountRotation.angleTo(rotation) > 1e-5
    )
      throw new Error("MuJoCo reduced FEM does not support rotating mounts");
    if (mesh.target !== undefined && mesh.target !== target)
      throw new Error("Reduced FEM mount target cannot change after joining");
    mesh.mountRotation ??= rotation;
    mesh.target = target;

    const size = model.dofs + 3;
    const mass = serial(model.mass, size);
    // Native moving bodies require positive mass. Subtract their exact contribution
    // before factoring, so this representation does not add artificial inertia.
    const epsilon = 1e-14;
    const residual = mass.slice();
    for (let i = 0; i < size; i++)
      for (let j = i % 3; j < size; j += 3)
        residual[i * size + j] =
          value(residual, i * size + j) - epsilon * (size - Math.max(i, j));
    const joints = Array.from({ length: size }, (_, i) => jointName(body, i));
    tendons.push(
      ...factors(
        cholesky(residual, size),
        joints,
        `${name(body)}m`,
        'armature="1"',
      ),
    );
    if (body.linearDamping > 0)
      tendons.push(
        ...factors(
          cholesky(mass, size),
          joints,
          `${name(body)}d`,
          `damping="${body.linearDamping}"`,
        ),
      );
    const elasticity = body.material.elasticity;
    const damping =
      "dampingTime" in elasticity ? (elasticity.dampingTime ?? 0) : 0;
    const stiffness = serial(model.stiffness, model.dofs, 0);
    tendons.push(
      ...factors(
        cholesky(stiffness, model.dofs),
        joints.slice(3),
        `${name(body)}k`,
        `stiffness="1" damping="${damping}" springlength="0"`,
      ),
    );

    // One slide per body prevents MuJoCo's simple-body optimization from dropping
    // cross-axis tendon armature terms; the serial tree retains every dense coupling.
    const chain: string[] = [];
    let previous = new Vector3();
    for (let i = 0; i < size; i++) {
      const offset = new Vector3();
      if (i >= 3 && i % 3 === 0) {
        const node = model.coordinates[(i - 3) / 3];
        if (node === undefined)
          throw new Error("Missing reduced FEM coordinate");
        const point = new Vector3().fromArray(model.positions, node * 3);
        offset.copy(point).sub(previous);
        previous = point;
      }
      const axis = [0, 0, 0];
      axis[i % 3] = 1;
      chain.push(
        `<body name="${name(body)}r${i}" ${i === 0 ? placement(pose) : `pos="${offset.toArray().join(" ")}"`} gravcomp="1"><inertial pos="0 0 0" mass="${epsilon}" diaginertia="1 1 1"/><joint name="${jointName(body, i)}" type="slide" axis="${axis.join(" ")}"/>${i === 2 ? `<site name="${name(body)}mount" pos="0 0 0"/>` : ""}`,
      );
    }
    bodies.push(chain.join("") + "</body>".repeat(size));
    if (!mesh.anchor) {
      mesh.anchor = new Vector3().setFromMatrixPosition(pose);
      if (target)
        mesh.anchor.applyMatrix4(
          splitTransform(target.matrixWorld).pose.invert(),
        );
    }
    const targetSites = sites.get(target) ?? [];
    targetSites.push(
      `<site name="${name(body)}anchor" pos="${mesh.anchor.toArray().join(" ")}"/>`,
    );
    sites.set(target, targetSites);
    equalities.push(
      `<connect site1="${name(body)}mount" site2="${name(body)}anchor" solref="${2 * timestep} 1" solimp="0.9999 0.9999 0.001"/>`,
    );

    const nodes = [...new Set(model.triangles)];
    const indices = new Map(nodes.map((node, i) => [node, i]));
    const owners: string[] = [],
      vertices: number[] = [];
    for (const node of nodes) {
      const coordinate = model.coordinates.indexOf(node);
      if (coordinate >= 0) {
        owners.push(`${name(body)}r${3 + coordinate * 3 + 2}`);
        vertices.push(0, 0, 0);
      } else if (model.mount.includes(node)) {
        owners.push(`${name(body)}r2`);
        vertices.push(...model.positions.slice(node * 3, node * 3 + 3));
      } else
        throw new Error(
          "Reduced FEM contact triangles must use retained or mounted vertices",
        );
    }
    const triangles = model.triangles.map((node) => {
      const index = indices.get(node);
      if (index === undefined)
        throw new Error("Missing reduced FEM contact vertex");
      return index;
    });
    const friction =
      body.material.dynamicFriction ?? body.material.staticFriction ?? 0.5;
    if ((body.material.staticFriction ?? friction) !== friction)
      throw new Error("MuJoCo requires equal static and dynamic friction");
    const geometry: Geometry = {
      name: name(body),
      owner: body,
      source: body,
      groups: { membership: 0xffff, filter: 0xffff },
      friction,
      torsionalFriction: body.material.torsionalFriction ?? 0,
      rollingFriction: body.material.rollingFriction ?? 0,
      restitution: body.material.restitution ?? 0,
    };
    flexes.push(
      `<flex name="${name(body)}" dim="2" radius="0" body="${owners.join(" ")}" vertex="${vertices.join(" ")}" element="${triangles.join(" ")}"><contact ${contact(geometry)} selfcollide="none"/></flex>`,
    );
  }
  return {
    bodies: bodies.join(""),
    flexes: flexes.join(""),
    tendons,
    equalities,
    sites,
  };
}

function factors(
  matrix: Float64Array,
  joints: readonly string[],
  prefix: string,
  attributes: string,
): string[] {
  return joints.map(
    (_, column) =>
      `<fixed name="${prefix}${column}" ${attributes}>${joints.map((joint, row) => `<joint joint="${joint}" coef="${value(matrix, row * joints.length + column)}"/>`).join("")}</fixed>`,
  );
}
