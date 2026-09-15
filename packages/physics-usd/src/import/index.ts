import { Object3D, Vector3 } from "three";
import type { LoadingManager } from "three";
import { USDComposer } from "three/addons/loaders/usd/USDComposer.js";
import { Joint, type PhysicsMaterial, RigidBody } from "@drawcall/physics";
import type { PhysicsWorld } from "@drawcall/physics";
import { PhysicsUSDScene } from "../scene.js";
import { attribute, boolean, numeric, schemas } from "./layer.js";
import {
  vector,
  wrapBody,
  ancestorBody,
  materialFor,
  massProperties,
} from "./bodies.js";
import { validate } from "./validate.js";
import { read } from "./read.js";
import { readJoint } from "./joints.js";
import { readShape } from "./shapes.js";

export interface PhysicsUSDImportOptions {
  manager?: LoadingManager;
  world?: PhysicsWorld;
}

export class PhysicsUSDLoader {
  constructor(private readonly options: PhysicsUSDImportOptions = {}) {}

  parse(input: ArrayBuffer | Uint8Array | string, path = ""): PhysicsUSDScene {
    return this.prepare(input, path).scene;
  }

  async parseAsync(
    input: ArrayBuffer | Uint8Array | string,
    path = "",
  ): Promise<PhysicsUSDScene> {
    const { scene, textures } = this.prepare(input, path);
    try {
      await Promise.all(textures);
      return scene;
    } catch (error) {
      scene.dispose();
      throw error;
    }
  }

  async loadAsync(url: string): Promise<PhysicsUSDScene> {
    const response = await fetch(url);
    if (!response.ok)
      throw new Error(`USD request failed: ${response.status} ${url}`);
    return this.parseAsync(
      await response.arrayBuffer(),
      new URL(".", response.url || url).href,
    );
  }

  private prepare(
    input: ArrayBuffer | Uint8Array | string,
    path: string,
  ): { scene: PhysicsUSDScene; textures: Promise<unknown>[] } {
    const { layer, assets } = read(input);
    validate(layer);
    const composer = new USDComposer(this.options.manager);
    const visual = composer.compose(layer, assets, {}, path);
    const scene = new PhysicsUSDScene(this.options.world);
    try {
      scene.add(...visual.children);
      const objects = new Map<string, Object3D>();
      const index = (object: Object3D, parent: string) => {
        const objectPath = `${parent}/${object.name}`;
        objects.set(objectPath, object);
        for (const child of object.children) index(child, objectPath);
      };
      for (const child of scene.children) index(child, "");
      for (const [primPath, object] of objects) {
        if (attribute(layer, primPath, "visibility") === "invisible")
          object.visible = false;
      }
      const bodies = new Map<string, RigidBody>();
      const materials = new Map<string, PhysicsMaterial>();
      for (const [primPath, spec] of Object.entries(layer.specsByPath)) {
        if (spec.specType !== 6) continue;
        const applied = schemas(layer, primPath);
        const rigid = applied.includes("PhysicsRigidBodyAPI");
        if (!rigid && !applied.includes("PhysicsMassAPI")) continue;
        const object = objects.get(primPath);
        if (!object)
          throw new Error(
            `Missing visual transform for rigid body ${primPath}`,
          );
        if (!rigid && ancestorBody(object)) continue;
        const body = wrapBody(
          object,
          scene.world,
          rigid && boolean(layer, primPath, "physics:rigidBodyEnabled", true)
            ? boolean(layer, primPath, "physics:kinematicEnabled", false)
              ? "kinematic"
              : "dynamic"
            : "static",
          massProperties(layer, primPath, object),
        );
        scene.own(body);
        const velocity = vector(
          layer,
          primPath,
          "physics:angularVelocity",
          [0, 0, 0],
        );
        body.setVelocity({
          linear: new Vector3(
            ...vector(layer, primPath, "physics:velocity", [0, 0, 0]),
          ),
          angular: new Vector3(...velocity).multiplyScalar(Math.PI / 180),
        });
        bodies.set(primPath, body);
      }
      for (const [primPath, spec] of Object.entries(layer.specsByPath)) {
        if (spec.specType !== 6) continue;
        const type = spec.fields.typeName;
        if (typeof type !== "string") continue;
        if (type === "PhysicsScene") {
          const direction = vector(
            layer,
            primPath,
            "physics:gravityDirection",
            [0, -1, 0],
          );
          const magnitude = numeric(
            layer,
            primPath,
            "physics:gravityMagnitude",
            9.81,
          );
          const length = Math.hypot(...direction);
          if (!length && magnitude)
            throw new Error("USD gravity direction cannot be zero");
          const factor = length ? magnitude / length : 0;
          scene.gravity = [
            direction[0] * factor,
            direction[1] * factor,
            direction[2] * factor,
          ];
          objects.get(primPath)?.removeFromParent();
          continue;
        }
        if (type.startsWith("Physics") && type.endsWith("Joint")) {
          const joint = readJoint(layer, primPath, type, bodies);
          scene.own(joint);
          joint.name = primPath.slice(primPath.lastIndexOf("/") + 1);
          const object = objects.get(primPath);
          const parent = object?.parent ?? scene;
          object?.removeFromParent();
          parent.add(joint);
          continue;
        }
        if (!schemas(layer, primPath).includes("PhysicsCollisionAPI")) continue;
        if (!boolean(layer, primPath, "physics:collisionEnabled", true))
          throw new Error(`Disabled colliders are not supported: ${primPath}`);
        const object = objects.get(primPath);
        if (!object) throw new Error(`Missing collision geometry ${primPath}`);
        let body = bodies.get(primPath) ?? ancestorBody(object);
        if (!body) {
          body = wrapBody(object, scene.world, "static");
          scene.own(body);
          bodies.set(primPath, body);
        }
        const material = materialFor(layer, primPath, materials);
        const collider = readShape(layer, primPath, type, object, material);
        const parent = object.parent;
        if (!parent) throw new Error(`Orphan collider ${primPath}`);
        parent.add(collider);
        // Collision-only geometry is invisible in the archive; visible geometry remains a visual child.
        if (!object.visible) object.removeFromParent();
      }
      for (const [primPath, object] of objects) {
        const name = layer.specsByPath[primPath]?.fields.displayName;
        if (typeof name === "string") {
          object.name = name;
          const body = bodies.get(primPath);
          if (body) body.name = name;
        }
      }
      scene.traverse((object) => {
        if (object instanceof RigidBody) object.getColliders();
        if (object instanceof Joint) object.validate();
      });
      return { scene, textures: composer.texturePromises };
    } catch (error) {
      scene.dispose();
      throw error;
    }
  }
}
