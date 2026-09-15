import { afterEach, beforeEach, describe, expect, it } from "vitest";
import {
  Matrix4,
  Object3D,
  BoxGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  Scene,
  Vector3,
} from "three";
import {
  clone,
  resolveCollider,
  AuthoringWorld,
  setDefaultWorld,
  getDefaultWorld,
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  DistanceJoint,
  FixedJoint,
  MeshCollider,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
  SphereCollider,
  SphericalJoint,
  Joint,
} from "@drawcall/physics";
import { strFromU8, unzipSync } from "fflate";
import {
  PhysicsUSDExporter,
  PhysicsUSDLoader,
  PhysicsUSDScene,
} from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
  setDefaultWorld(world);
});
afterEach(() => world.dispose());

function doorAssembly() {
  const scene = new Scene();
  const assembly = new Group();
  assembly.position.set(2, 3, 4);
  assembly.rotation.y = 0.6;
  scene.add(assembly);
  const frame = new RigidBody({ type: "static" });
  frame.name = "Frame";
  const material = new MeshStandardMaterial({ color: "brown" });
  for (const x of [-0.55, 0.55]) {
    const side = new Mesh(new BoxGeometry(0.1, 2.2, 0.15), material);
    side.position.set(x, 1.1, 0);
    frame.add(side);
  }
  const top = new Mesh(new BoxGeometry(1.2, 0.1, 0.15), material);
  top.position.set(0, 2.25, 0);
  frame.add(top);
  const door = new RigidBody({ mass: 20 });
  door.name = "Door";
  door.position.set(0, 1.05, 0);
  door.add(new Mesh(new BoxGeometry(0.98, 2, 0.06), material));
  const hinge = new RevoluteJoint({
    body0: frame,
    body1: door,
  }).setLimits([0, Math.PI / 2]);
  hinge.position.set(-0.49, 1.05, 0);
  assembly.add(frame, door, hinge);
  return { scene, frame, door, hinge };
}

describe("USD Physics interchange", () => {
  it("owns imported registrations without replacing the application world", async () => {
    const { scene } = doorAssembly();
    const imported = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    const bodies = imported
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof RigidBody);
    expect(bodies.length).toBe(2);
    for (const body of bodies) expect(body.world).toBe(imported.world);
    expect(imported.world).not.toBe(world);
    expect(getDefaultWorld()).toBe(world);
    imported.dispose();
    expect(getDefaultWorld()).toBe(world);
    expect(() => new RigidBody({ world: imported.world })).toThrow();
  });

  it("registers in a supplied world and disposes only imported objects", async () => {
    const { scene, door } = doorAssembly();
    const imported = new PhysicsUSDLoader({ world }).parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    const importedBodies = imported
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof RigidBody);
    for (const body of importedBodies) {
      expect(body.world).toBe(world);
      expect(world.objects.has(body)).toBe(true);
    }
    imported.dispose();
    for (const body of importedBodies) expect(body.disposed).toBe(true);
    expect(world.objects.has(door)).toBe(true);
    expect(door.disposed).toBe(false);
    expect(getDefaultWorld()).toBe(world);
  });

  it("cleans up failed imports without disposing a supplied world", () => {
    const existing = new RigidBody({ type: "static" });
    expect(() =>
      new PhysicsUSDLoader({ world }).parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Cube "Crate" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
)
{
 double size = 1
}
def PhysicsRevoluteJoint "Hinge"
{
 rel physics:body1 = </Crate>
 uniform token physics:axis = "INVALID"
}`),
    ).toThrow("Invalid joint axis");
    expect([...world.objects]).toEqual([existing]);
    expect(getDefaultWorld()).toBe(world);
  });

  it("imports without an installed default world", () => {
    world.dispose();
    const imported = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Cube "Crate" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
)
{
 double size = 1
}`);
    expect(
      imported
        .getObjectsByProperty("isObject3D", true)
        .filter((object) => object instanceof RigidBody),
    ).toHaveLength(1);
    expect(() => getDefaultWorld()).toThrow();
    imported.dispose();
  });

  it("roundtrips a transformed door, visual meshes, compound shapes, and references", async () => {
    const { scene, door, hinge } = doorAssembly();
    const before = new Vector3().setFromMatrixPosition(
      hinge.getFrame(0, new Matrix4()),
    );
    const bytes = await new PhysicsUSDExporter().parseAsync(scene);
    const result = await new PhysicsUSDLoader().parseAsync(bytes);
    const bodies = result
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof RigidBody);
    const joints = result
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof Joint);
    expect(bodies).toHaveLength(2);
    expect(joints).toHaveLength(1);
    const frame = bodies.find((body) => body.name === "Frame");
    const loadedDoor = bodies.find((body) => body.name === "Door");
    expect(frame?.getColliders()).toHaveLength(3);
    expect(loadedDoor?.options.mass).toBe(20);
    expect(
      loadedDoor
        ?.getWorldPosition(new Vector3())
        .distanceTo(door.getWorldPosition(new Vector3())),
    ).toBeLessThan(1e-5);
    const joint = joints[0];
    if (!(joint instanceof RevoluteJoint))
      throw new Error("Missing revolute joint");
    expect(joint.options.body0).toBe(frame);
    expect(joint.options.body1).toBe(loadedDoor);
    expect(joint.limits).toEqual(hinge.limits);
    expect(
      new Vector3()
        .setFromMatrixPosition(joint.getFrame(0, new Matrix4()))
        .distanceTo(before),
    ).toBeLessThan(1e-5);
    let meshes = 0;
    result.traverse((object) => {
      if (object instanceof Mesh) meshes++;
    });
    expect(meshes).toBe(4);
    expect(result.gravity).toEqual([0, -9.81, 0]);
    expect(door.parent?.children).toContain(hinge);
  });

  it("preserves camera prim types through the overlay", async () => {
    const { scene } = doorAssembly();
    const camera = new PerspectiveCamera(42);
    camera.name = "Camera";
    scene.add(camera);
    const result = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    expect(result.getObjectByName("Camera")).toBeInstanceOf(PerspectiveCamera);
  });

  it("roundtrips every V1 joint and explicit primitive shape", async () => {
    const scene = new Scene();
    const body = new RigidBody({ colliders: false });
    const material = {
      staticFriction: 0.8,
      dynamicFriction: 0.3,
      density: 1200,
      restitution: 0.2,
    };
    body.add(
      new BoxCollider({ size: [1, 2, 3] }).setMaterial(material),
      new SphereCollider({ radius: 0.4 }),
      new CapsuleCollider({ radius: 0.2, length: 1 }),
      new CylinderCollider({ radius: 0.3, height: 0.8 }),
      new MeshCollider({
        approximation: "convexHull",
      }).setGeometry(new BoxGeometry()),
    );
    const options = { body0: null, body1: body };
    scene.add(
      body,
      new FixedJoint(options),
      new SphericalJoint(options),
      new DistanceJoint(options).setLimits([0, 2]),
      new PrismaticJoint({
        ...options,
        axis: "Z",
      }).setLimits([-1, 1]),
    );
    const result = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    expect(
      result
        .getObjectsByProperty("isObject3D", true)
        .filter((object) => object instanceof Joint)
        .map((joint) => joint.constructor.name),
    ).toEqual([
      "FixedJoint",
      "SphericalJoint",
      "DistanceJoint",
      "PrismaticJoint",
    ]);
    const colliders = result
      .getObjectsByProperty("isObject3D", true)
      .find((object) => object instanceof RigidBody)
      ?.getColliders();
    expect(colliders?.map((collider) => collider.shape().kind)).toEqual([
      "box",
      "sphere",
      "capsule",
      "cylinder",
      "mesh",
    ]);
    expect(colliders?.slice(0, 4).map((collider) => collider.shape())).toEqual([
      { kind: "box", size: [1, 2, 3] },
      { kind: "sphere", radius: 0.4 },
      { kind: "capsule", radius: 0.2, length: 1 },
      { kind: "cylinder", radius: 0.3, height: 0.8 },
    ]);
    expect(colliders?.[0]?.material).toMatchObject(material);
  });

  it("imports independently authored standard USDA, including a mesh carrying a body schema", () => {
    const result = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
 upAxis = "Y"
)
def Cube "Crate" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
)
{
 double size = 2
 float physics:mass = 7
 double3 xformOp:translate = (1, 2, 3)
 uniform token[] xformOpOrder = ["xformOp:translate"]
}`);
    const body = result
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof RigidBody)[0];
    expect(body?.options.mass).toBe(7);
    expect(body?.getWorldPosition(new Vector3()).toArray()).toEqual([1, 2, 3]);
    expect(body?.getColliders()[0]?.position.toArray()).toEqual([0, 0, 0]);
    expect(body?.getColliders()[0]?.shape()).toEqual({
      kind: "box",
      size: [2, 2, 2],
    });
  });

  it("writes standard schemas, degrees and layer composition rather than serialized JS state", async () => {
    const { scene } = doorAssembly();
    const files = unzipSync(await new PhysicsUSDExporter().parseAsync(scene));
    const main = files["model.usda"];
    if (!main) throw new Error("Missing layer");
    const text = strFromU8(main);
    expect(text).toContain("subLayers = [@visuals.usda@]");
    expect(text).toContain('"PhysicsRigidBodyAPI"');
    expect(text).toContain("physics:upperLimit = 90");
    expect(text).not.toContain("drive:");
    expect(text).not.toContain("PhysicsDriveAPI");
    expect(text).toContain('displayName = "Door"');
    expect(text).not.toContain("glts:");
  });

  it("rejects unsupported export settings instead of dropping semantics", async () => {
    const { scene, door } = doorAssembly();
    door.setGravityScale(0);
    await expect(new PhysicsUSDExporter().parseAsync(scene)).rejects.toThrow(
      "gravity scale",
    );
    door.setGravityScale(1);
    door.add(
      new BoxCollider().setCollisionGroups({ membership: 1, filter: 2 }),
    );
    await expect(new PhysicsUSDExporter().parseAsync(scene)).rejects.toThrow(
      "collision filter",
    );
  });

  it("rejects unsupported stage units and backend physics schemas", () => {
    expect(() =>
      new PhysicsUSDLoader().parse("#usda 1.0\n(\n metersPerUnit = 0.01\n)"),
    ).toThrow("metersPerUnit=1");
    expect(() =>
      new PhysicsUSDLoader().parse(
        '#usda 1.0\n(\n metersPerUnit = 1\n)\ndef Xform "Body" (\n prepend apiSchemas = ["PhysicsArticulationRootAPI"]\n)\n{\n}',
      ),
    ).toThrow("Unsupported USD schema");
  });
  it("preserves world anchors when exporting an attached subtree", async () => {
    const outer = new Group();
    outer.position.x = 10;
    const scene = new Group();
    outer.add(scene);
    const body = new RigidBody({ colliders: false });
    body.add(new BoxCollider());
    const joint = new FixedJoint({ body0: null, body1: body });
    scene.add(body, joint);
    const result = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    expect(
      result
        .getObjectsByProperty("isObject3D", true)
        .filter((object) => object instanceof RigidBody)[0]
        ?.getWorldPosition(new Vector3()).x,
    ).toBe(10);
    expect(
      result
        .getObjectsByProperty("isObject3D", true)
        .filter((object) => object instanceof Joint)[0]
        ?.getFrame(0, new Matrix4()).elements[12],
    ).toBe(10);
  });

  it("rejects reset transform stacks and inline metadata", () => {
    const header = "#usda 1.0\n(\n metersPerUnit = 1\n)\n";
    expect(() =>
      new PhysicsUSDLoader().parse(
        header +
          'def Xform "Body" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { }',
      ),
    ).toThrow("multiline");
    expect(() =>
      new PhysicsUSDLoader().parse(
        header +
          'def Xform "Body"\n{\n uniform token[] xformOpOrder = ["!resetXformStack!", "xformOp:translate"]\n double3 xformOp:translate = (2, 0, 0)\n}',
      ),
    ).toThrow("transform operation");
  });

  it("retains physics material values across colliders", async () => {
    const scene = new Scene();
    const body = new RigidBody({ colliders: false });
    const material = { density: 234, staticFriction: 0.8 };
    body.add(
      new BoxCollider().setMaterial(material),
      new SphereCollider().setMaterial(material),
    );
    scene.add(body);
    const result = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(scene),
    );
    const colliders = result
      .getObjectsByProperty("isObject3D", true)
      .filter((object) => object instanceof RigidBody)[0]
      ?.getColliders();
    expect(colliders?.[0]?.material).toEqual(colliders?.[1]?.material);
  });
  it("retains imported gravity by default and permits an explicit export override", async () => {
    const { scene } = doorAssembly();
    const exporter = new PhysicsUSDExporter();
    const loader = new PhysicsUSDLoader();
    for (const gravity of [
      [0, 0, 0],
      [2, -3, 4],
    ]) {
      const [x, y, z] = gravity;
      if (x === undefined || y === undefined || z === undefined)
        throw new Error("Missing test vector");
      const imported = loader.parse(
        await exporter.parseAsync(scene, { gravity: [x, y, z] }),
      );
      const reimported = loader.parse(await exporter.parseAsync(imported));
      expect(reimported.gravity[0]).toBeCloseTo(x);
      expect(reimported.gravity[1]).toBeCloseTo(y);
      expect(reimported.gravity[2]).toBeCloseTo(z);
      const overridden = loader.parse(
        await exporter.parseAsync(imported, { gravity: [0, -1, 0] }),
      );
      expect(overridden.gravity).toEqual([0, -1, 0]);
    }
  });
});

it("rejects merging independent physics worlds into one USD simulation", async () => {
  const other = new AuthoringWorld();
  try {
    const scene = new Group();
    for (const target of [world, other]) {
      const body = new RigidBody({ world: target });
      body.add(new Mesh(new BoxGeometry(), new MeshStandardMaterial()));
      scene.add(body);
    }
    await expect(new PhysicsUSDExporter().parseAsync(scene)).rejects.toThrow(
      "different physics worlds",
    );
  } finally {
    other.dispose();
  }
});

it("rejects orphan colliders and joints referencing bodies outside the export", async () => {
  const exporter = new PhysicsUSDExporter();
  const scene = new Group();
  const collider = new BoxCollider();
  scene.add(collider);
  await expect(exporter.parseAsync(scene)).rejects.toThrow(
    "Collider must belong",
  );
  collider.removeFromParent();
  const outside = new RigidBody({ type: "static" });
  const joint = new FixedJoint({ body0: null, body1: outside });
  scene.add(joint);
  await expect(exporter.parseAsync(scene)).rejects.toThrow(
    "Joint body missing",
  );
});

it("clones imported assemblies with remapped joints and independent disposal", async () => {
  const { scene } = doorAssembly();
  const imported = new PhysicsUSDLoader().parse(
    await new PhysicsUSDExporter().parseAsync(scene),
  );
  const native = imported.clone();
  const nativeJoint = native
    .getObjectsByProperty("isObject3D", true)
    .find((object) => object instanceof Joint);
  if (!(nativeJoint instanceof Joint))
    throw new Error("Missing native cloned joint");
  expect(nativeJoint.options.body1).toBe(imported.getObjectByName("Door"));
  native.dispose();
  const cloned = clone(imported);
  expect(cloned.world).toBe(imported.world);
  expect(cloned.gravity).toEqual(imported.gravity);
  const originalDoor = imported.getObjectByName("Door");
  const copiedDoor = cloned.getObjectByName("Door");
  expect(copiedDoor).not.toBe(originalDoor);
  const joints = cloned
    .getObjectsByProperty("isObject3D", true)
    .filter((object) => object instanceof Joint);
  expect(joints).toHaveLength(1);
  expect(joints[0]?.options.body1).toBe(copiedDoor);
  copiedDoor?.removeFromParent();
  cloned.dispose();
  if (
    !(originalDoor instanceof RigidBody) ||
    !(copiedDoor instanceof RigidBody)
  )
    throw new Error("Missing cloned door bodies");
  expect(copiedDoor.disposed).toBe(true);
  expect(originalDoor.disposed).toBe(false);
  expect(
    () => new RigidBody({ world: imported.world, type: "static" }),
  ).not.toThrow();
  imported.dispose();
});

it("exports static groups without rigid-body schemas and preserves their compound colliders", async () => {
  const scene = new Group();
  const floor = new RigidBody({ type: "static" });
  floor.name = "Floor";
  floor.position.set(2, 3, 4);
  floor.add(new Mesh(new BoxGeometry()));
  const second = new Mesh(new BoxGeometry());
  second.position.x = 2;
  floor.add(second);
  scene.add(floor);
  const bytes = await new PhysicsUSDExporter().parseAsync(scene);
  const layer = unzipSync(bytes)["model.usda"];
  if (!layer) throw new Error("Missing USD layer");
  const text = strFromU8(layer);
  expect(text).not.toContain("PhysicsRigidBodyAPI");
  expect(text).not.toContain("physics:velocity");
  expect(text).not.toContain("physics:kinematicEnabled");
  expect(text).toContain("PhysicsCollisionAPI");
  const loaded = new PhysicsUSDLoader().parse(bytes);
  try {
    const copy = loaded.getObjectByName("Floor");
    if (!(copy instanceof RigidBody)) throw new Error("Missing static group");
    expect(copy.options.type).toBe("static");
    expect(copy.getColliders()).toHaveLength(2);
    expect(copy.getWorldPosition(new Vector3()).toArray()).toEqual([2, 3, 4]);
  } finally {
    loaded.dispose();
  }
});

it("roundtrips scaled visuals, colliders and joint anchors without accumulating scale", async () => {
  const { scene, door, hinge } = doorAssembly();
  const assembly = door.parent;
  if (!assembly) throw new Error("Missing assembly");
  assembly.scale.setScalar(2);
  door.scale.set(1, 2, 1);
  const mesh = door.children[0];
  if (!(mesh instanceof Mesh)) throw new Error("Missing door mesh");
  mesh.scale.setScalar(0.5);
  scene.updateMatrixWorld(true);
  const matrix = mesh.matrixWorld.clone();
  const anchors = [
    hinge.getFrame(0, new Matrix4()),
    hinge.getFrame(1, new Matrix4()),
  ];
  let source: Object3D = assembly;
  for (let iteration = 0; iteration < 2; iteration++) {
    const result = new PhysicsUSDLoader().parse(
      await new PhysicsUSDExporter().parseAsync(source),
    );
    result.updateMatrixWorld(true);
    const loaded = result.getObjectsByProperty("isObject3D", true);
    const body = loaded.find(
      (object) => object instanceof RigidBody && object.name === "Door",
    );
    const joint = loaded.find((object) => object instanceof RevoluteJoint);
    if (!(body instanceof RigidBody) || !(joint instanceof RevoluteJoint))
      throw new Error("Missing physics");
    const visual = body.children.find((object) => object instanceof Mesh);
    if (!visual) throw new Error("Missing visual");
    visual.matrixWorld.elements.forEach((value, index) =>
      expect(value).toBeCloseTo(matrix.elements[index] ?? Infinity, 5),
    );
    const collider = body.getColliders()[0];
    if (!collider) throw new Error("Missing collider");
    const shape = resolveCollider(body, collider).shape;
    if (shape.kind !== "box") throw new Error("Expected box");
    shape.size.forEach((value, index) =>
      expect(value).toBeCloseTo([0.98, 4, 0.06][index] ?? Infinity, 5),
    );
    for (const index of [0, 1] as const) {
      const expected = anchors[index];
      if (!expected) throw new Error("Missing anchor");
      joint
        .getFrame(index, new Matrix4())
        .elements.forEach((value, i) =>
          expect(value).toBeCloseTo(expected.elements[i] ?? Infinity, 5),
        );
    }
    if (source instanceof PhysicsUSDScene) source.dispose();
    source = result;
  }
  if (source instanceof PhysicsUSDScene) source.dispose();
  expect(assembly.scale.toArray()).toEqual([2, 2, 2]);
  expect(door.scale.toArray()).toEqual([1, 2, 1]);
  expect(mesh.scale.toArray()).toEqual([0.5, 0.5, 0.5]);
});
