import { expect, it } from "vitest";
import { BoxGeometry, Group, Matrix4, Mesh, Vector3 } from "three";
import { FixedJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "../src/index.js";

it("captures scale only at the first step, including bodies added and removed during simulation", async () => {
  const world = await setupWorld();
  try {
    const floor = new RigidBody({ type: "static" });
    floor.add(new Mesh(new BoxGeometry(30, 1, 30)));
    floor.position.y = -0.5;
    const spawn = (x: number, factor: number) => {
      const body = new RigidBody({ mass: 2 });
      const controls = world.body(body);
      expect(() => controls.getMatrix()).toThrow("first physics step");
      world.reset();
      expect(() => controls.getMatrix()).toThrow("first physics step");
      const mesh = new Mesh(new BoxGeometry());
      body.add(mesh);
      mesh.scale.setScalar(0.5);
      body.scale.setScalar(2);
      body.position.set(x / factor, 5 / factor, 0);
      const root = new Group().add(body);
      root.scale.setScalar(factor);
      return { body, root, controls };
    };
    const first = spawn(-4, 2);
    for (let i = 0; i < 180; i++) world.step();
    expect(first.body.getWorldPosition(new Vector3()).y).toBeCloseTo(1, 1);
    expect(first.body.scale.toArray()).toEqual([2, 2, 2]);
    first.body.dispose();
    expect(() => first.controls.getMatrix()).toThrow("not part");
    const second = spawn(4, 3);
    for (let i = 0; i < 180; i++) world.step();
    expect(second.body.getWorldPosition(new Vector3()).y).toBeCloseTo(1.5, 1);
    const pending = spawn(0, 4);
    pending.body.dispose();
    world.step();
    expect(() => pending.controls.getMatrix()).toThrow("not part");
    second.root.scale.setScalar(4);
    expect(() => world.step()).toThrow("scale cannot change");
  } finally {
    world.dispose();
  }
});

it("keeps explicit mass and derives density mass and inertia from scaled geometry", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  try {
    for (const mass of [undefined, 2]) {
      const body = new RigidBody({ mass, material: { density: 1 } });
      body.add(new Mesh(new BoxGeometry()));
      body.scale.setScalar(2);
      world.step();
      const controls = world.body(body);
      controls.applyImpulse(new Vector3(8, 0, 0));
      expect(controls.getVelocity().linear.x).toBeCloseTo(mass ? 4 : 1);
      controls.applyImpulse(new Vector3(0, 1, 0), new Vector3(1, 0, 0));
      expect(controls.getVelocity().angular.z).toBeCloseTo(
        1 / (((mass ?? 8) * 2) / 3),
      );
      body.dispose();
    }
  } finally {
    world.dispose();
  }
});

it("captures scaled joints added during simulation and preserves scale through teleport and reset", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  try {
    world.step();
    const root = new Group();
    root.scale.setScalar(2);
    root.rotation.y = 0.3;
    const body = new RigidBody();
    body.scale.set(1, 2, 1);
    body.position.y = 3;
    body.add(new Mesh(new BoxGeometry()));
    root.add(body);
    const joint = new FixedJoint({
      body0: null,
      body1: body,
      frame0: new Matrix4().makeTranslation(0, 10, 0),
      frame1: new Matrix4().makeTranslation(0, 1, 0),
    });
    const controls = world.joint(joint);
    expect(() => controls.getState()).toThrow("not active");
    for (let i = 0; i < 60; i++) world.step();
    expect(controls.getState().distance).toBeLessThan(0.001);
    expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(6);
    joint.dispose();
    world.body(body).teleport(new Matrix4().makeTranslation(0, 2, 0));
    expect(body.scale.toArray()).toEqual([1, 2, 1]);
    world.reset();
    expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(6);
    expect(body.scale.toArray()).toEqual([1, 2, 1]);
  } finally {
    world.dispose();
  }
});

it("captures new collider scale while rejecting edits to an existing collider's captured scale", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  try {
    const body = new RigidBody({ mass: 2 });
    const mesh = new Mesh(new BoxGeometry());
    body.add(mesh);
    world.update(world.fixedDelta / 2);
    mesh.scale.setScalar(2);
    world.update(world.fixedDelta / 2);
    mesh.scale.setScalar(3);
    expect(() => world.step()).toThrow("Collider scale cannot change");
    body.remove(mesh);
    const replacement = new Mesh(new BoxGeometry());
    replacement.scale.setScalar(3);
    body.add(replacement);
    world.step();
    world.body(body).applyImpulse(new Vector3(2, 0, 0));
    expect(world.body(body).getVelocity().linear.x).toBeCloseTo(1);
  } finally {
    world.dispose();
  }
});

it("preserves scale authored directly in a manual body matrix", async () => {
  const world = await setupWorld();
  try {
    const floor = new RigidBody({ type: "static" });
    floor.add(new Mesh(new BoxGeometry(20, 1, 20)));
    floor.position.y = -0.5;
    const body = new RigidBody();
    body.add(new Mesh(new BoxGeometry()));
    body.matrixAutoUpdate = false;
    body.matrix.makeScale(2, 2, 2).setPosition(0, 5, 0);
    body.matrixWorldNeedsUpdate = true;
    for (let i = 0; i < 180; i++) world.step();
    expect(
      body.getWorldScale(new Vector3()).distanceTo(new Vector3(2, 2, 2)),
    ).toBeLessThan(1e-6);
    expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(1, 1);
    world.reset();
    expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(5);
    expect(
      body.getWorldScale(new Vector3()).distanceTo(new Vector3(2, 2, 2)),
    ).toBeLessThan(1e-6);
  } finally {
    world.dispose();
  }
});
