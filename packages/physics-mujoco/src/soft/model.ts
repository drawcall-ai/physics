import {
  SoftAttachment,
  splitTransform,
  type RigidBody,
} from "@drawcall/physics";
import { selectSoftNodes } from "@drawcall/physics/backend";
import { Vector3 } from "three";
import { name } from "../model/markup.js";
import type { Geometry } from "../model/shapes.js";
import type { SoftMesh } from "./mesh.js";
import type { MjModel } from "@mujoco/mujoco";
import { array, at } from "../heap.js";

export function correctMembraneStiffness(model: MjModel): void {
  // MuJoCo 3.16 divides triangle stretching stiffness by four; tetrahedra cancel
  // this with a default thickness of four. Restore SI modulus without changing bending.
  // https://github.com/google-deepmind/mujoco/blob/3.16.0/src/user/user_flexcomp.cc#L1338-L1368
  const stiffness = array(model.flex_stiffness);
  for (let flex = 0; flex < model.nflex; flex++) {
    if (at(model.flex_dim, flex) !== 2 || at(model.flex_interp, flex)) continue;
    const start = at(model.flex_stiffnessadr, flex);
    if (start < 0) continue;
    const end = start + 21 * at(model.flex_elemnum, flex);
    for (let i = start; i < end; i++) stiffness[i] = 4 * at(stiffness, i);
  }
}

export interface Attachment {
  object: SoftAttachment;
  nodes: number[];
  anchors: Vector3[];
}
export function attachment(
  object: SoftAttachment,
  mesh: SoftMesh,
  positions: ArrayLike<number>,
): Attachment {
  const nodes = selectSoftNodes(object, mesh.local);
  const target = object.options.target;
  target?.updateWorldMatrix(true, false);
  const inverse = target && splitTransform(target.matrixWorld).pose.invert();
  const anchors = nodes.map((node) => {
    const point = new Vector3().fromArray(positions, node * 3);
    return inverse ? point.applyMatrix4(inverse) : point;
  });
  return { object, nodes, anchors };
}
export function nodeName(mesh: SoftMesh, node: number): string {
  return `${name(mesh.body)}n${node}`;
}

export function softXml(
  meshes: Iterable<SoftMesh>,
  attachments: Iterable<Attachment>,
  contact: (geometry: Geometry) => string,
  fixedDelta: number,
) {
  const bodies: string[] = [],
    flexes: string[] = [],
    tendons: string[] = [],
    equalities: string[] = [];
  const sites = new Map<RigidBody | null, string[]>();
  for (const mesh of meshes) {
    const { body, dimension, radius, rest, volumes } = mesh;
    const material = body.material;
    const friction = material.dynamicFriction ?? material.staticFriction ?? 0.5;
    if ((material.staticFriction ?? friction) !== friction)
      throw new Error("MuJoCo requires equal static and dynamic friction");
    const elastic = material.elasticity;
    if (dimension === 1) {
      if (elastic.model !== "spring")
        throw new Error("MuJoCo curves require a spring material");
      if (elastic.bend)
        throw new Error("MuJoCo curve bending is not supported");
      for (let i = 0; i < mesh.elements.length; i += 2) {
        const a = mesh.elements[i],
          b = mesh.elements[i + 1];
        if (a === undefined || b === undefined)
          throw new Error("Incomplete soft edge");
        const va = volumes[a],
          vb = volumes[b];
        if (va === undefined || vb === undefined)
          throw new Error("Unknown soft node");
        // Frequency uses the two rest-node masses' reduced mass, independent of constraints.
        const reducedMass = (material.density * va * vb) / (va + vb);
        const omega = 2 * Math.PI * elastic.stretch.frequency;
        const length = new Vector3()
          .fromArray(rest, a * 3)
          .distanceTo(new Vector3().fromArray(rest, b * 3));
        tendons.push(
          `<spatial name="${name(body)}e${i}" stiffness="${omega ** 2 * reducedMass}" damping="${2 * elastic.stretch.dampingRatio * omega * reducedMass}" springlength="${length}"><site site="${nodeName(mesh, a)}site"/><site site="${nodeName(mesh, b)}site"/></spatial>`,
        );
      }
    } else {
      if (
        elastic.model !== "saintVenantKirchhoff" &&
        !(dimension === 3 && elastic.model === "neoHookean")
      )
        throw new Error(
          "MuJoCo surfaces require saintVenantKirchhoff; volumes also support neoHookean elasticity",
        );
      if (elastic.model === "neoHookean" && elastic.dampingRatio)
        throw new Error(
          "MuJoCo neoHookean elasticity does not support dampingRatio",
        );
    }
    const nodes: string[] = [];
    for (let node = 0; node < volumes.length; node++) {
      const key = nodeName(mesh, node);
      nodes.push(key);
      const volume = volumes[node];
      if (volume === undefined) throw new Error("Missing soft node volume");
      const mass = volume * material.density;
      // XYZ particles never rotate, but MuJoCo still requires inertia >= mjMINVAL.
      const inertia = Math.max(mass * 1e-6, 1e-15);
      const joints = ["1 0 0", "0 1 0", "0 0 1"]
        .map(
          (axis, i) =>
            `<joint name="${key}j${i}" type="slide" axis="${axis}"/>`,
        )
        .join("");
      bodies.push(
        `<body name="${key}" pos="${Array.from(rest.slice(node * 3, node * 3 + 3)).join(" ")}">${joints}<inertial pos="0 0 0" mass="${mass}" diaginertia="${inertia} ${inertia} ${inertia}"/><site name="${key}site" pos="0 0 0" size="0.001"/></body>`,
      );
    }
    const geometry: Geometry = {
      name: name(body),
      owner: body,
      source: body,
      groups: { membership: 0xffff, filter: 0xffff },
      friction,
      torsionalFriction: material.torsionalFriction ?? 0,
      rollingFriction: material.rollingFriction ?? 0,
      restitution: material.restitution ?? 0,
    };
    let elasticity = "";
    if (
      elastic.model === "saintVenantKirchhoff" ||
      elastic.model === "neoHookean"
    ) {
      let poisson = elastic.poissonRatio;
      let young = elastic.youngModulus;
      if (elastic.model === "saintVenantKirchhoff") {
        // MuJoCo 3.16 doubles the SVK lambda term. Halve lambda while preserving mu.
        // https://github.com/google-deepmind/mujoco/blob/3.16.0/src/user/user_flexcomp.cc#L1232-L1236
        poisson /= 2 * (1 - poisson);
        young *= (1 + poisson) / (1 + elastic.poissonRatio);
      }
      const damping = elastic.dampingTime ?? 0;
      elasticity = `<elasticity young="${young}" poisson="${poisson}" damping="${damping}"${dimension === 2 ? ` thickness="${mesh.thickness}" elastic2d="both"` : ""}/>`;
    }
    flexes.push(
      `<flex name="${name(body)}" dim="${dimension}" radius="${radius}" body="${nodes.join(" ")}" element="${mesh.elements.join(" ")}">${elasticity}<contact ${contact(geometry)} selfcollide="${body.options.selfCollision ? "auto" : "none"}"/></flex>`,
    );
  }
  const pinned = new Set<string>();
  for (const { object, nodes, anchors } of attachments) {
    if (!object.enabled) continue;
    const target = object.options.target;
    const list = sites.get(target) ?? [];
    sites.set(target, list);
    for (const [i, node] of nodes.entries()) {
      const nodeKey = `${name(object.options.body)}n${node}`;
      if (pinned.has(nodeKey))
        throw new Error("Soft attachments cannot share simulation nodes");
      pinned.add(nodeKey);
      const point = anchors[i];
      if (!point) throw new Error("Missing attachment anchor");
      const key = `${name(object)}a${i}`;
      list.push(
        `<site name="${key}" pos="${point.toArray().join(" ")}" size="0.001"/>`,
      );
      // Independent particles retain stiffness preconditioning; maximal impedance limits clamp drift.
      equalities.push(
        `<connect name="${key}" site1="${name(object.options.body)}n${node}site" site2="${key}" solref="${fixedDelta * 2} 1" solimp="0.9999 0.9999 0.001"/>`,
      );
    }
  }
  return {
    bodies: bodies.join(""),
    flexes: flexes.join(""),
    tendons,
    equalities,
    sites,
  };
}
