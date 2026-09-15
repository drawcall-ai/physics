import { Group, Matrix4, Object3D, Scene } from "three";
import { USDZExporter } from "three/addons/exporters/USDZExporter.js";
import type { USDZExporterOptions } from "three/addons/exporters/USDZExporter.js";
import { strToU8, unzipSync } from "fflate";
import {
  Collider,
  Joint,
  RigidBody,
  resolveCollider,
  splitTransform,
} from "@drawcall/physics";
import type { PhysicsMaterial, Vec3 } from "@drawcall/physics";
import { writeJoint } from "./joints.js";
import { PhysicsUSDScene } from "../scene.js";
import { archive } from "./archive.js";
import { Prim } from "./prim.js";
import { shapePrim, tuple } from "./shapes.js";

export interface PhysicsUSDExportOptions extends USDZExporterOptions {
  gravity?: Vec3;
}
const degrees = 180 / Math.PI;

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
    const bodies: RigidBody[] = [];
    const jointObjects: Joint[] = [];
    scene.updateWorldMatrix(true, true);
    scene.traverse((object) => {
      if (object instanceof RigidBody) bodies.push(object);
      if (object instanceof Joint) jointObjects.push(object);
      if (!(object instanceof Collider)) return;
      let parent = object.parent;
      while (parent && !(parent instanceof RigidBody)) parent = parent.parent;
      if (!parent || !bodies.some((body) => body === parent))
        throw new Error(
          "Collider must belong to a rigid body in the exported scene",
        );
    });
    const world = bodies[0]?.world;
    if (bodies.some((body) => body.world !== world))
      throw new Error(
        "Cannot export bodies from different physics worlds into one USD simulation",
      );
    const paths = new Map<Object3D, string>();
    const prims = new Map<Object3D, Prim>();
    const root = new Prim("Root");
    const scenes = new Prim("Scenes", "Scope");
    const stage = new Prim("Scene");
    root.children.push(scenes);
    scenes.children.push(stage);
    let next = 0;
    const copy = (
      object: Object3D,
      parent: Prim,
      parentPath: string,
    ): Object3D | undefined => {
      if (object instanceof Collider || object instanceof Joint)
        return undefined;
      const name = `P${next++}`;
      const clone =
        object instanceof RigidBody ||
        object instanceof Scene ||
        object instanceof PhysicsUSDScene
          ? new Group().copy(object, false)
          : object.clone(false);
      clone.name = name;
      clone.matrixAutoUpdate = false;
      clone.matrix.copy(object.matrix);
      if (object instanceof RigidBody) {
        clone.matrix.copy(splitTransform(object.matrixWorld).pose);
        if (object.parent)
          clone.matrix.premultiply(object.parent.matrixWorld.clone().invert());
      }
      if (object.parent instanceof RigidBody)
        clone.matrix.premultiply(
          new Matrix4().makeScale(
            ...splitTransform(object.parent.matrixWorld).scale.toArray(),
          ),
        );
      const prim = new Prim(name, "");
      prim.displayName = object.name;
      if (!object.visible)
        prim.properties.push('token visibility = "invisible"');
      parent.children.push(prim);
      paths.set(object, `${parentPath}/${name}`);
      prims.set(object, prim);
      for (const child of object.children) {
        const result = copy(child, prim, `${parentPath}/${name}`);
        if (result) clone.add(result);
      }
      return clone;
    };
    const visuals = new Scene();
    const clone = copy(scene, stage, "/Root/Scenes/Scene");
    if (!clone)
      throw new Error("Cannot export a collider or joint as the scene root");
    clone.matrix.copy(
      scene instanceof RigidBody
        ? splitTransform(scene.matrixWorld).pose
        : scene.matrixWorld,
    );
    visuals.add(clone);
    visuals.updateMatrixWorld(true);
    const materials = new Prim("PhysicsMaterials", "Scope");
    const materialPaths = new Map<PhysicsMaterial | undefined, string>();
    for (const body of bodies) {
      const colliders = body.getColliders();
      const object = body;
      const prim = prims.get(object);
      if (!prim) throw new Error("Body missing from USD hierarchy");
      if (
        object.linearDamping !== 0 ||
        object.angularDamping !== 0 ||
        object.gravityScale !== 1 ||
        !(object.options.canSleep ?? true)
      )
        throw new Error(
          "Core USD Physics cannot represent damping, gravity scale, or sleep policy overrides",
        );
      prim.schemas.push("PhysicsMassAPI");
      if (object.bodyType !== "static") {
        prim.schemas.push("PhysicsRigidBodyAPI");
        prim.properties.push(
          "bool physics:rigidBodyEnabled = true",
          `bool physics:kinematicEnabled = ${object.bodyType === "kinematic"}`,
          `vector3f physics:velocity = ${tuple(object.getVelocity().linear.toArray())}`,
          `vector3f physics:angularVelocity = ${tuple(
            object
              .getVelocity()
              .angular.toArray()
              .map((value) => value * degrees),
          )}`,
        );
      }
      if (object.options.mass !== undefined)
        prim.properties.push(`float physics:mass = ${object.options.mass}`);
      for (const name of ["centerOfMass", "diagonalInertia"] as const) {
        const value = object.options[name];
        if (value)
          prim.properties.push(
            `${name === "centerOfMass" ? "point3f" : "float3"} physics:${name} = ${tuple(value)}`,
          );
      }
      const axes = object.options.principalAxes;
      if (axes)
        prim.properties.push(
          `quatf physics:principalAxes = ${tuple([axes[3], axes[0], axes[1], axes[2]])}`,
        );
      for (const [index, collider] of colliders.entries()) {
        if (collider.collisionGroups)
          throw new Error(
            "USD collision filter conversion is not implemented; remove collisionGroups or export without physics",
          );
        if (collider.sensor)
          throw new Error("Core USD Physics cannot represent sensors");
        const physicsMaterial = body.getMaterial(collider);
        const materialKey = collider.material ?? body.material;
        let materialPath = materialPaths.get(materialKey);
        if (!materialPath) {
          const material = new Prim(
            `Material${materialPaths.size}`,
            "Material",
          );
          material.schemas.push("PhysicsMaterialAPI");
          for (const property of [
            "staticFriction",
            "dynamicFriction",
            "restitution",
            "density",
          ] as const)
            material.properties.push(
              `float physics:${property} = ${physicsMaterial[property]}`,
            );
          materials.children.push(material);
          materialPath = `/Root/PhysicsMaterials/${material.name}`;
          materialPaths.set(materialKey, materialPath);
        }
        const resolved = resolveCollider(body, collider);
        const shape = shapePrim(
          `Collider${index}`,
          resolved.shape,
          resolved.matrix,
        );
        shape.schemas.push("PhysicsCollisionAPI", "MaterialBindingAPI");
        shape.properties.push(
          "bool physics:collisionEnabled = true",
          'token visibility = "invisible"',
          `rel material:binding:physics = <${materialPath}>`,
        );
        prim.children.push(shape);
      }
    }
    const joints = new Prim("PhysicsJoints", "Scope");
    for (const [index, joint] of jointObjects.entries())
      joints.children.push(writeJoint(joint, `Joint${index}`, paths));
    const physicsScene = new Prim("PhysicsScene", "PhysicsScene");
    const gravity =
      options.gravity ??
      (scene instanceof PhysicsUSDScene ? scene.gravity : [0, -9.81, 0]);
    const magnitude = Math.hypot(...gravity);
    if (!gravity.every(Number.isFinite))
      throw new Error("Gravity must be finite");
    physicsScene.properties.push(
      `vector3f physics:gravityDirection = ${tuple(magnitude ? gravity.map((value) => value / magnitude) : [0, -1, 0])}`,
      `float physics:gravityMagnitude = ${magnitude}`,
    );
    const visualArchive = await this.visual.parseAsync(visuals, {
      ...options,
      onlyVisible: false,
    });
    const files = unzipSync(new Uint8Array(visualArchive));
    const visualLayer = files["model.usda"];
    if (!visualLayer)
      throw new Error("Three USDZExporter did not produce model.usda");
    delete files["model.usda"];
    root.children.push(materials, joints, physicsScene);
    const layer =
      '#usda 1.0\n(\n defaultPrim = "Root"\n metersPerUnit = 1\n kilogramsPerUnit = 1\n upAxis = "Y"\n subLayers = [@visuals.usda@]\n)\n' +
      [root].map((prim) => prim.write()).join("\n");
    return archive({
      "model.usda": strToU8(layer),
      "visuals.usda": visualLayer,
      ...files,
    });
  }
}
