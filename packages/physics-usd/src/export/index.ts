import { Object3D, Scene } from "three";
import { USDZExporter } from "three/addons/exporters/USDZExporter.js";
import type { USDZExporterOptions } from "three/addons/exporters/USDZExporter.js";
import { strToU8, unzipSync } from "fflate";
import {
  Collider,
  Joint,
  RigidBody,
  ancestorBody,
  splitTransform,
} from "@drawcall/physics";
import type { Vec3 } from "@drawcall/physics";
import { PhysicsUSDScene } from "../scene.js";
import { archive } from "./archive.js";
import { Materials, writeBody } from "./bodies.js";
import { Hierarchy } from "./hierarchy.js";
import { writeJoint } from "./joints.js";
import { Prim } from "./prim.js";
import { tuple } from "./shapes.js";

export interface PhysicsUSDExportOptions extends USDZExporterOptions {
  gravity?: Vec3;
}

export class PhysicsUSDExporter {
  private readonly visual = new USDZExporter();

  setTextureUtils(utils: Parameters<USDZExporter["setTextureUtils"]>[0]): void {
    this.visual.setTextureUtils(utils);
  }

  async parseAsync(
    scene: Object3D,
    options: PhysicsUSDExportOptions = {},
  ): Promise<Uint8Array<ArrayBuffer>> {
    if (options.animations?.length)
      throw new Error(
        "Physics USD export does not yet support animation clips",
      );
    if (options.onlyVisible === true)
      throw new Error(
        "Physics USD export includes hidden collision geometry; onlyVisible=true is unsupported",
      );
    const { bodies, joints } = collect(scene);
    const root = new Prim("Root");
    const scenes = new Prim("Scenes", "Scope");
    const stage = new Prim("Scene");
    root.children.push(scenes);
    scenes.children.push(stage);
    const hierarchy = new Hierarchy();
    const clone = hierarchy.mirror(scene, stage, "/Root/Scenes/Scene");
    if (!clone)
      throw new Error("Cannot export a collider or joint as the scene root");
    clone.matrix.copy(
      scene instanceof RigidBody
        ? splitTransform(scene.matrixWorld).pose
        : scene.matrixWorld,
    );
    const visuals = new Scene();
    visuals.add(clone);
    visuals.updateMatrixWorld(true);
    const materials = new Materials();
    for (const body of bodies) writeBody(hierarchy, body, materials);
    const jointScope = new Prim("PhysicsJoints", "Scope");
    for (const [index, joint] of joints.entries())
      jointScope.children.push(
        writeJoint(joint, `Joint${index}`, hierarchy.paths),
      );
    const gravity =
      options.gravity ??
      (scene instanceof PhysicsUSDScene ? scene.gravity : [0, -9.81, 0]);
    root.children.push(materials.scope, jointScope, scenePrim(gravity));
    const files = unzipSync(
      new Uint8Array(
        await this.visual.parseAsync(visuals, {
          ...options,
          onlyVisible: false,
        }),
      ),
    );
    const visualLayer = files["model.usda"];
    if (!visualLayer)
      throw new Error("Three USDZExporter did not produce model.usda");
    delete files["model.usda"];
    const layer =
      '#usda 1.0\n(\n defaultPrim = "Root"\n metersPerUnit = 1\n kilogramsPerUnit = 1\n upAxis = "Y"\n subLayers = [@visuals.usda@]\n)\n' +
      root.write();
    return archive({
      "model.usda": strToU8(layer),
      "visuals.usda": visualLayer,
      ...files,
    });
  }
}

/** Bodies and joints in traversal order; every collider must sit under one of the bodies. */
function collect(scene: Object3D): { bodies: RigidBody[]; joints: Joint[] } {
  const bodies: RigidBody[] = [];
  const joints: Joint[] = [];
  scene.updateWorldMatrix(true, true);
  scene.traverse((object) => {
    if (object instanceof RigidBody) bodies.push(object);
    if (object instanceof Joint) joints.push(object);
    if (!(object instanceof Collider)) return;
    const body = ancestorBody(object);
    if (!body || !bodies.includes(body))
      throw new Error(
        "Collider must belong to a rigid body in the exported scene",
      );
  });
  const world = bodies[0]?.world;
  if (bodies.some((body) => body.world !== world))
    throw new Error(
      "Cannot export bodies from different physics worlds into one USD simulation",
    );
  return { bodies, joints };
}

function scenePrim(gravity: Vec3): Prim {
  if (!gravity.every(Number.isFinite))
    throw new Error("Gravity must be finite");
  const magnitude = Math.hypot(...gravity);
  const prim = new Prim("PhysicsScene", "PhysicsScene");
  prim.properties.push(
    `vector3f physics:gravityDirection = ${tuple(magnitude ? gravity.map((value) => value / magnitude) : [0, -1, 0])}`,
    `float physics:gravityMagnitude = ${magnitude}`,
  );
  return prim;
}
