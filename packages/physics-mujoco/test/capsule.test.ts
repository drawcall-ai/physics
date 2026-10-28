import { expect, test, vi } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  SphereCollider,
  Trigger,
} from "@drawcall/physics";
import {
  BoxGeometry,
  BufferGeometry,
  Float32BufferAttribute,
  Matrix4,
  Vector3,
} from "three";
import { body, createWorld, ground, scene } from "./fixtures.js";
import { capsuleContacts } from "../src/capsule.js";
import { compile } from "../src/model/compile.js";
import { modelOptions } from "../src/options.js";
import { array } from "../src/heap.js";

const shape = { radius: 0.4, height: 1 };
const pose = (x: number, y: number, z = 0) =>
  new Matrix4().makeTranslation(x, y, z);

test("capsule contacts report signed distance, obstacle points and outward normals", async () => {
  const floor = ground();
  const world = await createWorld();
  for (const y of [1, 0.9, 0.8]) {
    const [contact] = world.capsuleContacts(shape, pose(0, y), 0.2);
    expect(contact?.body).toBe(floor);
    expect(contact?.object).toBe(floor.children[0]);
    expect(contact?.distance).toBeCloseTo(y - 0.9, 6);
    expect(contact?.point.y).toBeCloseTo(0, 6);
    expect(contact?.normal.y).toBeCloseTo(1, 6);
  }
  expect(world.capsuleContacts(shape, pose(0, 1), 0.05)).toEqual([]);
});

test("capsule sweeps catch thin walls and floors at high speed", async () => {
  const floor = ground();
  const wall = new RigidBody({ bodyType: "static" }).add(
    new BoxCollider({ size: [0.02, 10, 10] }),
  );
  wall.position.x = 3;
  scene.add(wall);
  const world = await createWorld();
  const hit = world.castCapsule(shape, pose(0, 2), new Vector3(100, 0, 0));
  expect(hit?.body).toBe(wall);
  expect(hit?.fraction).toBeCloseTo((3 - 0.01 - shape.radius) / 100, 6);
  expect(hit?.normal.x).toBeCloseTo(-1, 6);
  const down = world.castCapsule(shape, pose(0, 2), new Vector3(0, -100, 0));
  expect(down?.body).toBe(floor);
  expect(down?.fraction).toBeCloseTo(1.1 / 100, 6);
  expect(world.castCapsule(shape, pose(0, 2), new Vector3())).toBeNull();
});

test("capsule queries follow authored static poses without a simulation step", async () => {
  const wall = body("static");
  wall.position.x = 2;
  const world = await createWorld();
  const start = pose(0, 0);
  const movement = new Vector3(10, 0, 0);
  expect(world.capsuleContacts(shape, start, 10)[0]?.distance).toBeCloseTo(
    1.1,
    6,
  );
  expect(world.castCapsule(shape, start, movement)?.fraction).toBeCloseTo(
    0.11,
    6,
  );
  wall.position.x = 4;
  expect(world.capsuleContacts(shape, start, 10)[0]?.distance).toBeCloseTo(
    3.1,
    6,
  );
  wall.position.x = 6;
  expect(world.castCapsule(shape, start, movement)?.fraction).toBeCloseTo(
    0.51,
    6,
  );
  expect(world.time).toBe(0);
});

test("initial overlaps block deeper motion but allow escape and tangent movement", async () => {
  ground();
  const world = await createWorld();
  const start = pose(0, 0.8);
  expect(world.castCapsule(shape, start, new Vector3(0, -1, 0))?.fraction).toBe(
    0,
  );
  expect(world.castCapsule(shape, start, new Vector3(0, 1, 0))).toBeNull();
  expect(world.castCapsule(shape, start, new Vector3(1, 0, 0))).toBeNull();
  expect(
    world.castCapsule(shape, pose(0, 0.9), new Vector3(1, 0, 0)),
  ).toBeNull();
});

test("capsule orientation and radius changes take effect without rebuilding", async () => {
  ground();
  const world = await createWorld();
  const horizontal = pose(0, 1).multiply(
    new Matrix4().makeRotationZ(Math.PI / 2),
  );
  expect(world.capsuleContacts(shape, horizontal, 1)[0]?.distance).toBeCloseTo(
    0.6,
    6,
  );
  expect(
    world.capsuleContacts({ radius: 2, height: 2 }, pose(0, 4), 2)[0]?.distance,
  ).toBeCloseTo(1, 6);
  expect(world.capsuleContacts(shape, pose(0, 1), 1)[0]?.distance).toBeCloseTo(
    0.1,
    6,
  );
});

test("queries respect collision groups and exclusions and ignore triggers", async () => {
  const wall = body("static");
  wall.position.x = 3;
  wall.setCollisionGroups({ membership: 1, filter: 2 });
  const trigger = new Trigger().add(new SphereCollider({ radius: 1 }));
  trigger.position.x = 1;
  scene.add(trigger);
  const world = await createWorld();
  const start = pose(0, 0);
  const movement = new Vector3(5, 0, 0);
  expect(world.castCapsule(shape, start, movement)?.body).toBe(wall);
  expect(
    world.castCapsule(shape, start, movement, { excludeBodies: [wall] }),
  ).toBeNull();
  expect(
    world.castCapsule(shape, start, movement, {
      collisionGroups: { membership: 4, filter: 1 },
    }),
  ).toBeNull();
  expect(
    world.castCapsule(shape, start, movement, {
      collisionGroups: { membership: 2, filter: 1 },
    })?.body,
  ).toBe(wall);
});

test("queries preserve physics state, contacts, and raycasts", async () => {
  ground();
  const box = body();
  box.position.set(1, 2, 0);
  const world = await createWorld();
  world.update(0.01);
  const before = box.position.clone();
  const velocity = box.getVelocity();
  const time = world.time;
  const ray = () =>
    world.raycast(new Vector3(0, 3, 0), new Vector3(0, -1, 0), 10);
  const distance = ray()?.distance;
  world.capsuleContacts({ radius: 5, height: 20 }, pose(0, 0), 2);
  world.castCapsule(shape, pose(0, 4), new Vector3(0, -20, 0));
  expect(world.time).toBe(time);
  expect(box.position.equals(before)).toBe(true);
  expect(box.getVelocity()).toEqual(velocity);
  expect(ray()?.distance).toBe(distance);
  expect(box.getContacts()).toEqual([]);
  world.update(0.01);
  expect(box.position.equals(before)).toBe(true);
});

test("sweeps follow rotated convex mesh obstacles", async () => {
  const geometry = new BoxGeometry(8, 0.2, 8);
  const ramp = new RigidBody({ bodyType: "static" }).add(
    new MeshCollider({ approximation: "convexHull" }).setGeometry(geometry),
  );
  ramp.rotation.z = 0.3;
  scene.add(ramp);
  try {
    const world = await createWorld();
    const hit = world.castCapsule(shape, pose(0, 3), new Vector3(0, -5, 0));
    expect(hit?.body).toBe(ramp);
    expect(hit?.normal.x).toBeCloseTo(-Math.sin(0.3), 4);
    expect(hit?.normal.y).toBeCloseTo(Math.cos(0.3), 4);
    expect(hit?.fraction).toBeGreaterThan(0.3);
    expect(hit?.fraction).toBeLessThan(0.5);
  } finally {
    geometry.dispose();
  }
});

test("unsupported heightfield capsule queries fail explicitly", async () => {
  const geometry = new BufferGeometry()
    .setAttribute(
      "position",
      new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1], 3),
    )
    .setIndex([0, 2, 1, 1, 2, 3]);
  const field = new RigidBody({ bodyType: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry),
  );
  scene.add(field);
  try {
    const world = await createWorld();
    expect(() =>
      world.castCapsule(shape, pose(0, 2), new Vector3(0, -3, 0)),
    ).toThrow("heightfields");
    expect(
      world.castCapsule(shape, pose(0, 2), new Vector3(0, -3, 0), {
        excludeBodies: [field],
      }),
    ).toBeNull();
  } finally {
    geometry.dispose();
  }
});

test("a failed native query restores its probe and preserves physical contacts", async () => {
  const floor = ground();
  const box = body();
  box.position.y = 0.49;
  scene.updateMatrixWorld(true);
  const api = await loadMujoco();
  const options = modelOptions(
    { scene },
    { fixedDelta: 0.01, gravity: [0, -9.81, 0], solverIterations: 50 },
  );
  const sim = compile(api, [floor, box], new Map(), [], options);
  const snapshot = () => ({
    size: array(sim.model.geom_size).slice(),
    bound: array(sim.model.geom_rbound).slice(),
    position: array(sim.data.geom_xpos).slice(),
    rotation: array(sim.data.geom_xmat).slice(),
    pose: array(sim.data.qpos).slice(),
    velocity: array(sim.data.qvel).slice(),
    contacts: sim.data.ncon,
  });
  const before = snapshot();
  expect(before.contacts).toBeGreaterThan(0);
  const native = vi.spyOn(api, "mj_geomDistance");
  try {
    native.mockImplementationOnce(() => {
      throw new Error("distance query failed");
    });
    expect(() => capsuleContacts(api, sim, shape, pose(3, 2), 10, {})).toThrow(
      "distance query failed",
    );
    expect(snapshot()).toEqual(before);
    expect(
      capsuleContacts(api, sim, shape, pose(0, 2), 2, {}).length,
    ).toBeGreaterThan(0);
    expect(snapshot()).toEqual(before);
  } finally {
    native.mockRestore();
    sim.free();
  }
});
