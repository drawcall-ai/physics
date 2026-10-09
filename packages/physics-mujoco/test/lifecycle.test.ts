import { expect, test, vi } from "vitest";
import {
  BoxCollider,
  MeshCollider,
  RigidBody,
  JointDrive,
  RevoluteJoint,
  Trigger,
} from "@drawcall/physics";
import {
  BoxGeometry,
  Group,
  Matrix4,
  Mesh,
  PerspectiveCamera,
  Vector3,
} from "three";
import { createWorld, scene } from "./fixtures.js";

test("unchanged mesh steps and target edits reuse collision geometry; marked vertex edits rebuild it", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  scene.add(body);
  const geometry = new BoxGeometry();
  body.add(new MeshCollider().setGeometry(geometry));
  const clone = vi.spyOn(geometry, "clone");
  const joint = new RevoluteJoint({ body0: null, body1: body });
  scene.add(joint);
  const drive = new JointDrive({ stiffness: 1, damping: 1 });
  joint.setDrive(drive);
  world.update(0);
  const initialCopies = clone.mock.calls.length;
  expect(initialCopies).toBeGreaterThan(0);
  for (let i = 0; i < 10; i++) {
    drive.setTarget({ position: i * 0.01 });
    world.update(world.fixedDelta);
  }
  expect(clone.mock.calls.length).toBe(initialCopies);
  const positions = geometry.getAttribute("position");
  for (let i = 0; i < positions.count; i++)
    positions.setX(i, positions.getX(i) * 3);
  positions.needsUpdate = true;
  world.update(0);
  expect(clone.mock.calls.length).toBeGreaterThan(initialCopies);
  const hit = world.raycast(new Vector3(3, 0, 0), new Vector3(-1, 0, 0), 5);
  expect(hit?.distance).toBeCloseTo(1.5, 2);
  geometry.setDrawRange(0, 3);
  expect(() => world.update(world.fixedDelta)).toThrow("full draw range");
  geometry.dispose();
});

test("unchanged steps skip the change scan; moved and added colliders are still picked up", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  scene.add(body);
  const holder = new Group();
  holder.add(new BoxCollider());
  body.add(holder);
  const joint = new RevoluteJoint({ body0: null, body1: body });
  scene.add(joint);
  const drive = new JointDrive({ stiffness: 1, damping: 1 });
  joint.setDrive(drive);
  world.update(0);
  const scan = vi.spyOn(body, "getColliders");
  for (let i = 0; i < 10; i++) {
    drive.setTarget({ position: i * 0.01 });
    world.update(world.fixedDelta);
  }
  expect(scan).not.toHaveBeenCalled();
  holder.position.x = 1;
  world.update(0);
  expect(scan).toHaveBeenCalled();
  const right = world.raycast(new Vector3(3, 0, 0), new Vector3(-1, 0, 0), 5);
  expect(right?.distance).toBeCloseTo(1.5, 2);
  const added = new BoxCollider();
  added.position.x = -2;
  body.add(added);
  world.update(0);
  const left = world.raycast(new Vector3(-4, 0, 0), new Vector3(1, 0, 0), 5);
  expect(left?.distance).toBeCloseTo(1.5, 2);
});

test("a body without explicit colliders picks up its moved mesh", async () => {
  const world = await createWorld();
  const body = new RigidBody({ bodyType: "static" });
  scene.add(body);
  const mesh = new Mesh(new BoxGeometry());
  body.add(mesh);
  world.update(0);
  const scan = vi.spyOn(body, "getColliders");
  world.update(world.fixedDelta);
  expect(scan).not.toHaveBeenCalled();
  mesh.position.x = 2;
  world.update(world.fixedDelta);
  expect(scan).toHaveBeenCalled();
  const hit = world.raycast(new Vector3(4, 0, 0), new Vector3(-1, 0, 0), 5);
  expect(hit?.distance).toBeCloseTo(1.5, 2);
  mesh.geometry.dispose();
});

test("moving a child trigger or camera does not rescan its body", async () => {
  const world = await createWorld();
  const body = new RigidBody({ bodyType: "kinematic" });
  scene.add(body);
  const trigger = new Trigger();
  trigger.add(new BoxCollider());
  const camera = new PerspectiveCamera();
  body.add(new BoxCollider(), trigger, camera);
  world.update(0);
  const scan = vi.spyOn(body, "getColliders");
  for (let i = 1; i <= 10; i++) {
    trigger.position.x = i;
    camera.lookAt(i, 0, 1);
    body.setKinematicTarget(new Matrix4().makeTranslation(i * 0.01, 0, 0));
    world.update(world.fixedDelta);
  }
  expect(scan).not.toHaveBeenCalled();
});

test("a failed rebuild keeps live state and retries after correction", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1 });
  scene.add(body);
  body.add(new BoxCollider());
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  const position = body.position.clone();
  body.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.8 });
  expect(() => world.update(world.fixedDelta)).toThrow(
    "equal static and dynamic friction",
  );
  expect(body.position.equals(position)).toBe(true);
  expect(body.getVelocity().linear.x).toBeCloseTo(2);
  expect(world.time).toBe(0.01);
  body.setMaterial({ staticFriction: 0.4, dynamicFriction: 0.4 });
  world.update(world.fixedDelta);
  expect(body.position.x).toBeCloseTo(0.04);
  expect(body.getVelocity().linear.x).toBeCloseTo(2);
});

test("failed initial compilation does not capture reset poses or scale", async () => {
  const world = await createWorld();
  const body = new RigidBody({ mass: 1, velocity: { linear: [1, 0, 0] } });
  scene.add(body);
  body.add(new BoxCollider());
  body.position.x = 1;
  body.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.8 });
  expect(() => world.update(0)).toThrow("equal static and dynamic friction");
  body.setMaterial({ staticFriction: 0.4, dynamicFriction: 0.4 });
  body.position.x = 2;
  body.scale.setScalar(2);
  world.update(world.fixedDelta);
  world.reset();
  expect(body.position.x).toBe(2);
  expect(body.getVelocity().linear.x).toBe(1);
});

/** A body whose join fails: MuJoCo needs equal static and dynamic friction. */
function unjoinable(): RigidBody {
  const body = new RigidBody({ mass: 1 }).add(new BoxCollider());
  body.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.8 });
  return body;
}

test("restarts a body that left its world from its initial velocity", async () => {
  const world = await createWorld();
  const leaving = new RigidBody({ mass: 1 }).add(new BoxCollider());
  scene.add(leaving);
  world.update(world.fixedDelta);
  leaving.applyImpulse(new Vector3(3, 0, 0));
  const simulated = leaving.getVelocity().linear;
  expect(simulated.x).toBeCloseTo(3);
  leaving.removeFromParent();
  world.update(0);
  expect(leaving.world).toBeUndefined();
  expect(() => leaving.getVelocity()).toThrow(
    "not under a built world's scene",
  );
  scene.add(leaving);
  expect(leaving.getVelocity().linear.toArray()).toEqual([0, 0, 0]);
});

test("a body leaves in the same refresh as a failed join", async () => {
  const world = await createWorld();
  const leaving = new RigidBody({ mass: 1 }).add(new BoxCollider());
  scene.add(leaving);
  leaving.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  leaving.removeFromParent();
  const bad = unjoinable();
  scene.add(bad);
  expect(() => world.update(world.fixedDelta)).toThrow("equal static");
  expect(leaving.world).toBeUndefined();
  expect(() => leaving.getVelocity()).toThrow(
    "not under a built world's scene",
  );
  bad.removeFromParent();
  world.update(world.fixedDelta);
  const ray = world.raycast(new Vector3(0, 5, 0), new Vector3(0, -1, 0), 10);
  expect(ray).toBeNull();
});

test("a body that left during a failed refresh rejoins from its options and scene pose", async () => {
  const world = await createWorld();
  const body = new RigidBody({
    mass: 1,
    velocity: { linear: [0, 0, 5] },
  }).add(new BoxCollider());
  scene.add(body);
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(world.fixedDelta);
  body.removeFromParent();
  const bad = unjoinable();
  scene.add(bad);
  expect(() => world.update(world.fixedDelta)).toThrow("equal static");
  body.position.set(10, 0, 0);
  scene.add(body);
  bad.removeFromParent();
  world.update(0);
  expect(body.getVelocity().linear.toArray()).toEqual([0, 0, 5]);
  expect(body.position.x).toBe(10);
});

test("a teleport never moves a body that left during a failed refresh", async () => {
  const world = await createWorld();
  const rejoined = new RigidBody({ mass: 1 }).add(new BoxCollider());
  const staying = new RigidBody({ mass: 1 }).add(new BoxCollider());
  staying.position.x = 3;
  scene.add(rejoined, staying);
  world.update(world.fixedDelta);
  rejoined.removeFromParent();
  const bad = unjoinable();
  scene.add(bad);
  expect(() => world.update(world.fixedDelta)).toThrow("equal static");
  rejoined.position.y = 10;
  scene.add(rejoined);
  bad.removeFromParent();
  staying.teleport(new Matrix4().makeTranslation(3, -5, 0));
  expect(rejoined.position.y).toBe(10);
  expect(staying.position.y).toBe(-5);
});
