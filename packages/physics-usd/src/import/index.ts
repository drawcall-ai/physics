import { Object3D, Vector3 } from "three";
import type { LoadingManager } from "three";
import { USDComposer } from "three/addons/loaders/usd/USDComposer.js";
import {
  rollback,
  Joint,
  RigidBody,
  ancestorBody,
  type PhysicsMaterial,
} from "@drawcall/physics";
import { PhysicsUSDScene } from "../scene.js";
import { radians } from "../units.js";
import { attribute, boolean, numeric, prims, schemas } from "./layer.js";
import type { Layer } from "./layer.js";
import {
  vector,
  wrapBody,
  bodyType,
  materialFor,
  massProperties,
} from "./bodies.js";
import { validate } from "./validate.js";
import { read } from "./read.js";
import { readJoint } from "./joints.js";
import { readShape } from "./shapes.js";

export interface PhysicsUSDImportOptions {
  manager?: LoadingManager;
}

export class PhysicsUSDLoader {
  constructor(private readonly options: PhysicsUSDImportOptions = {}) {}

  /** Textures keep loading after this returns and report through the `LoadingManager`; `parseAsync` waits for them. */
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
      rollback(
        error,
        [() => scene.dispose()],
        "Physics operation and cleanup failed",
      );
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
    const scene = new PhysicsUSDScene();
    try {
      scene.add(...visual.children);
      new LayerImport(layer, scene).run();
      return { scene, textures: composer.texturePromises };
    } catch (error) {
      rollback(
        error,
        [() => scene.dispose()],
        "Physics operation and cleanup failed",
      );
    }
  }
}

/** Turns the physics prims of a validated layer into the bodies, colliders, and joints of one scene. */
class LayerImport {
  /** Composed visual objects by prim path. */
  private readonly objects = new Map<string, Object3D>();
  private readonly bodies = new Map<string, RigidBody>();
  private readonly materials = new Map<string, PhysicsMaterial>();

  constructor(
    private readonly layer: Layer,
    private readonly scene: PhysicsUSDScene,
  ) {}

  run(): void {
    for (const child of this.scene.children) this.index(child, "");
    for (const [path, object] of this.objects) {
      if (attribute(this.layer, path, "visibility") === "invisible")
        object.visible = false;
    }
    for (const [path] of prims(this.layer)) this.readBody(path);
    for (const [path, spec] of prims(this.layer)) {
      const type = spec.fields.typeName;
      if (typeof type !== "string") continue;
      if (type === "PhysicsScene") this.readScene(path);
      else if (type.startsWith("Physics") && type.endsWith("Joint"))
        this.placeJoint(path, type);
      else this.readCollider(path, type);
    }
    this.applyDisplayNames();
    this.scene.traverse((object) => {
      // Deriving colliders validates their geometry; the result is recreated on demand.
      if (object instanceof RigidBody) object.getColliders();
      if (object instanceof Joint) object.validate();
    });
  }

  private index(object: Object3D, parent: string): void {
    const path = `${parent}/${object.name}`;
    this.objects.set(path, object);
    for (const child of object.children) this.index(child, path);
  }

  private readBody(path: string): void {
    const applied = schemas(this.layer, path);
    const rigid = applied.includes("PhysicsRigidBodyAPI");
    if (!rigid && !applied.includes("PhysicsMassAPI")) return;
    const object = this.objects.get(path);
    if (!object)
      throw new Error(`Missing visual transform for rigid body ${path}`);
    if (!rigid && ancestorBody(object)) return;
    const body = wrapBody(
      object,
      bodyType(this.layer, path, rigid),
      massProperties(this.layer, path, object),
    );
    this.scene.own(body);
    const angular = vector(
      this.layer,
      path,
      "physics:angularVelocity",
      [0, 0, 0],
    );
    const velocity = {
      linear: new Vector3(
        ...vector(this.layer, path, "physics:velocity", [0, 0, 0]),
      ),
      angular: new Vector3(...angular).multiplyScalar(radians),
    };
    if (body.bodyType === "dynamic") body.setVelocity(velocity);
    else if (velocity.linear.lengthSq() || velocity.angular.lengthSq())
      throw new Error(`Velocity on a non-dynamic body is unsupported: ${path}`);
    this.bodies.set(path, body);
  }

  private readScene(path: string): void {
    const direction = vector(
      this.layer,
      path,
      "physics:gravityDirection",
      [0, -1, 0],
    );
    const magnitude = numeric(
      this.layer,
      path,
      "physics:gravityMagnitude",
      9.81,
    );
    const length = Math.hypot(...direction);
    if (!length && magnitude)
      throw new Error("USD gravity direction cannot be zero");
    const factor = length ? magnitude / length : 0;
    this.scene.gravity = [
      direction[0] * factor,
      direction[1] * factor,
      direction[2] * factor,
    ];
    this.objects.get(path)?.removeFromParent();
  }

  /** The joint takes the place of its visual transform, or hangs off the scene without one. */
  private placeJoint(path: string, type: string): void {
    const joint = readJoint(this.layer, path, type, this.bodies);
    this.scene.own(joint);
    joint.name = path.slice(path.lastIndexOf("/") + 1);
    const object = this.objects.get(path);
    const parent = object?.parent ?? this.scene;
    object?.removeFromParent();
    parent.add(joint);
  }

  /** Collision geometry outside any body gets a static body of its own. */
  private readCollider(path: string, type: string): void {
    if (!schemas(this.layer, path).includes("PhysicsCollisionAPI")) return;
    if (!boolean(this.layer, path, "physics:collisionEnabled", true))
      throw new Error(`Disabled colliders are not supported: ${path}`);
    const object = this.objects.get(path);
    if (!object) throw new Error(`Missing collision geometry ${path}`);
    let body = this.bodies.get(path) ?? ancestorBody(object);
    if (!body) {
      body = wrapBody(object, "static");
      this.scene.own(body);
      this.bodies.set(path, body);
    }
    const material = materialFor(this.layer, path, this.materials);
    const collider = readShape(this.layer, path, type, object, material);
    const parent = object.parent;
    if (!parent) throw new Error(`Orphan collider ${path}`);
    parent.add(collider);
    // Collision-only geometry is invisible in the archive; visible geometry remains a visual child.
    if (!object.visible) object.removeFromParent();
  }

  private applyDisplayNames(): void {
    for (const [path, object] of this.objects) {
      const name = this.layer.specsByPath[path]?.fields.displayName;
      if (typeof name !== "string") continue;
      object.name = name;
      const body = this.bodies.get(path);
      if (body) body.name = name;
    }
  }
}
