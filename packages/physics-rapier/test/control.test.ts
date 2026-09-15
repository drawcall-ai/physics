import { describe, expect, it } from "vitest";
import { BoxGeometry, PlaneGeometry, Matrix4, Mesh, Vector3 } from "three";
import {
  BoxCollider,
  MeshCollider,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { setupWorld } from "../src/index.js";

function body() {
  return new RigidBody({
    colliders: false,
    mass: 1,
    diagonalInertia: [1, 1, 1],
  });
}

describe("joint controls", () => {
  it("replaces effort, preserves body force, and consumes it for only one substep", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
    const moving = body();
    const slider = new PrismaticJoint({
      body0: null,
      body1: moving,
      axis: "X",
    });
    slider.setEffort(2);
    slider.setEffort(3);
    world.update(0);
    moving.applyForce(new Vector3(4, 0, 0));
    world.update(0.004);
    expect(world.time).toBe(0);
    world.update(0.006);
    expect(slider.getState().velocity).toBeCloseTo(0.07, 5);
    world.update(0.02);
    expect(slider.getState().velocity).toBeCloseTo(0.07, 5);
    slider.setEffort(9);
    slider.setEffort(0);
    world.update(0.01);
    expect(slider.getState().velocity).toBeCloseTo(0.07, 5);
    slider.setEffort(9);
    slider.setEnabled(false);
    slider.setEnabled(true);
    world.update(0.01);
    expect(slider.getState().velocity).toBeCloseTo(0.07, 5);
    world.dispose();
  });

  it("applies opposing torques in the joint frame on the first substep", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
    const first = body(),
      second = body();
    const hinge = new RevoluteJoint({ body0: first, body1: second, axis: "Z" });
    hinge.setEffort(2);
    world.update(0.01);
    expect(first.getVelocity().angular.z).toBeCloseTo(-0.02, 5);
    expect(second.getVelocity().angular.z).toBeCloseTo(0.02, 5);
    expect(hinge.getState().velocity).toBeCloseTo(0.04, 5);
    world.dispose();
  });

  it("commands a newly assembled joint from a before-step observer", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
    let moving: RigidBody | undefined;
    const unsubscribe = world.onBeforeStep(() => {
      moving = body();
      new PrismaticJoint({ body0: null, body1: moving, axis: "X" }).setEffort(
        5,
      );
      unsubscribe();
    });
    world.update(0.01);
    expect(moving?.getVelocity().linear.x).toBeCloseTo(0.05, 5);
    world.dispose();
  });

  it("measures rotating reference axes and velocities at offset centers of mass", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.0001 });
    const first = body().setVelocity({ angular: new Vector3(0, 0, 2) });
    const second = new RigidBody({
      colliders: false,
      mass: 1,
      centerOfMass: [1, 0, 0],
      diagonalInertia: [1, 1, 1],
    }).setVelocity({ angular: new Vector3(0, 0, 3) });
    second.position.set(2, 1, 0);
    const slider = new PrismaticJoint({
      body0: first,
      body1: second,
      axis: "X",
      frame0: new Matrix4().makeTranslation(0, 1, 0),
      frame1: new Matrix4().makeTranslation(0, 2, 0),
    }).setEnabled(false);
    world.update(0);
    const initial = slider.getState();
    expect(initial.velocity).toBeCloseTo(0, 6);
    world.update(world.fixedDelta);
    expect(
      (slider.getState().position - initial.position) / world.fixedDelta,
    ).toBeCloseTo(initial.velocity, 2);
    const comSlider = new PrismaticJoint({
      body0: null,
      body1: second,
      axis: "Y",
      frame0: new Matrix4(),
      frame1: new Matrix4(),
    }).setEnabled(false);
    world.update(0);
    expect(comSlider.getState().velocity).toBeCloseTo(-3, 4);
    world.dispose();
  });

  it("uses setters at construction and runtime while retaining the initialized velocity baseline", async () => {
    const world = await setupWorld({ gravity: [0, -10, 0], fixedDelta: 0.01 });
    const moving = body()
      .setVelocity({ linear: new Vector3(2, 0, 0) })
      .setGravityScale(0)
      .setLinearDamping(0);
    world.update(0.01);
    expect(moving.getVelocity().linear.toArray()).toEqual([2, 0, 0]);
    moving
      .setVelocity({ linear: new Vector3(4, 0, 0) })
      .setLinearDamping(10)
      .setGravityScale(1);
    world.update(0.01);
    expect(moving.getVelocity().linear.x).toBeLessThan(4);
    expect(moving.getVelocity().linear.y).toBeLessThan(0);
    world.reset();
    expect(moving.getVelocity().linear.toArray()).toEqual([2, 0, 0]);
    world.dispose();
  });

  it("tracks turns without getters and rebases teleport/reset", async () => {
    for (const speed of [8, -8]) {
      const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
      const moving = body().setVelocity({ angular: new Vector3(0, 0, speed) });
      const hinge = new RevoluteJoint({
        body0: null,
        body1: moving,
        axis: "Z",
      });
      for (let i = 0; i < 200; i++) world.update(0.01);
      expect(hinge.getState().position).toBeCloseTo(speed * 2, 1);
      moving.teleport(new Matrix4().makeRotationZ(0.25));
      expect(hinge.getState().position).toBeCloseTo(0.25, 5);
      world.reset();
      expect(hinge.getState().position).toBeCloseTo(0, 5);
      expect(world.time).toBe(0);
      world.dispose();
    }
  });
});

describe("simulation time", () => {
  it("commits completed steps before observers and never replays a failed observer interval", async () => {
    const world = await setupWorld({
      gravity: [0, 0, 0],
      fixedDelta: 0.01,
      maxSubsteps: 2,
    });
    const observed: number[] = [];
    world.onBeforeStep(() => observed.push(world.time));
    const unsubscribe = world.onAfterStep(() => {
      throw new Error("observer");
    });
    expect(() => world.update(0.01)).toThrow("observer");
    expect(world.time).toBeCloseTo(0.01);
    unsubscribe();
    world.update(0);
    expect(observed).toEqual([0]);
    world.update(1);
    expect(world.time).toBeCloseTo(0.03);
    expect(observed).toEqual([0, 0.01, 0.02]);
    world.dispose();
  });
});

describe("mass and queries", () => {
  it("rejects zero-density dynamic bodies and preserves an initialized body after invalid material changes", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0] });
    const moving = new RigidBody().setMaterial({ density: 0 });
    moving.add(new BoxCollider());
    expect(() => world.update(0)).toThrow("positive mass");
    moving.setMaterial({ density: 1 });
    world.update(0);
    moving.setMaterial({ density: 0 });
    expect(() => world.update(world.fixedDelta)).toThrow("positive mass");
    moving.applyImpulse(new Vector3(1, 0, 0));
    expect(moving.getVelocity().linear.x).toBeCloseTo(1, 5);
    world.dispose();
  });

  it("uses complete mass properties without deriving volume from surface colliders", async () => {
    const world = await setupWorld();
    new RigidBody({
      type: "static",
      mass: 1,
      centerOfMass: [0, 0, 0],
      diagonalInertia: [1, 1, 1],
      principalAxes: [0, 0, 0, 1],
    }).add(
      new MeshCollider({ approximation: "trimesh" }).setGeometry(
        new PlaneGeometry(1, 1),
      ),
    );
    expect(() => world.update(0)).not.toThrow();
    world.dispose();
  });

  it("retains density-weighted COM when overriding only total mass", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0] });
    const moving = new RigidBody({ mass: 10 });
    const light = new BoxCollider().setMaterial({ density: 1 });
    const heavy = new BoxCollider().setMaterial({ density: 9 });
    heavy.position.x = 4;
    moving.add(light, heavy);
    world.update(0);
    moving.applyImpulse(new Vector3(0, 10, 0), new Vector3(3.6, 0, 0));
    expect(moving.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(moving.getVelocity().angular.length()).toBeLessThan(1e-6);
    world.dispose();
  });

  it("preserves authoritative inertia through geometry edits and applies COM offset", async () => {
    const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
    const moving = new RigidBody({
      mass: 2,
      centerOfMass: [1, 0, 0],
      diagonalInertia: [2, 2, 2],
    });
    const shape = new Mesh(new BoxGeometry(1, 1, 1));
    moving.add(shape);
    world.update(0);
    moving.applyImpulse(new Vector3(0, 2, 0), new Vector3(0, 0, 0));
    expect(moving.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(moving.getVelocity().angular.z).toBeCloseTo(-1, 5);
    moving.setVelocity({ linear: new Vector3(), angular: new Vector3() });
    shape.geometry = new BoxGeometry(4, 4, 4);
    world.update(world.fixedDelta);
    moving.applyImpulse(new Vector3(0, 2, 0), new Vector3(0, 0, 0));
    expect(moving.getVelocity().linear.y).toBeCloseTo(1, 5);
    expect(moving.getVelocity().angular.z).toBeCloseTo(-1, 5);
    world.dispose();
  });

  it("queries only prepared bodies and current teleports, including inside exits and source identity", async () => {
    const world = await setupWorld();
    const moving = new RigidBody({ type: "static" });
    const mesh = new Mesh(new BoxGeometry(2, 2, 2));
    moving.add(mesh);
    const origin = new Vector3(-3, 0, 0),
      direction = new Vector3(5, 0, 0);
    expect(world.raycast(origin, direction, 10)).toBeNull();
    world.update(0);
    const hit = world.raycast(origin, direction, 10);
    expect(hit?.distance).toBeCloseTo(2);
    expect(hit?.normal.toArray()).toEqual([-1, 0, 0]);
    expect(hit?.body).toBe(moving);
    expect(hit?.collider).toBe(mesh);
    expect(world.raycast(new Vector3(), direction, 10)?.distance).toBeCloseTo(
      1,
    );
    expect(
      world.raycast(origin, direction, 10, { excludeBodies: [moving] }),
    ).toBeNull();
    moving.teleport(new Matrix4().makeTranslation(2, 0, 0));
    expect(world.raycast(origin, direction, 10)?.distance).toBeCloseTo(4);
    expect(world.time).toBe(0);
    world.dispose();
  });

  it("filters sensors and interaction groups", async () => {
    const world = await setupWorld();
    const sensor = new RigidBody({ type: "static", colliders: false });
    sensor.add(
      new BoxCollider()
        .setSensor(true)
        .setCollisionGroups({ membership: 2, filter: 4 }),
    );
    world.update(0);
    const origin = new Vector3(-3, 0, 0),
      direction = new Vector3(1, 0, 0);
    expect(world.raycast(origin, direction, 10)).toBeNull();
    expect(
      world.raycast(origin, direction, 10, {
        includeSensors: true,
        collisionGroups: { membership: 4, filter: 2 },
      })?.body,
    ).toBe(sensor);
    expect(
      world.raycast(origin, direction, 10, {
        includeSensors: true,
        collisionGroups: { membership: 4, filter: 1 },
      }),
    ).toBeNull();
    world.dispose();
  });
});

it("normalizes engine-derived masses when explicit mass is paired with zero density", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const moving = new RigidBody({ mass: 3 }).setMaterial({ density: 0 });
  moving.add(new BoxCollider());
  const larger = new BoxCollider().setSize([2, 1, 1]);
  larger.position.x = 3;
  moving.add(larger);
  world.update(0);
  moving.applyImpulse(new Vector3(0, 3, 0), new Vector3(2, 0, 0));
  expect(moving.getVelocity().linear.y).toBeCloseTo(1);
  expect(moving.getVelocity().angular.length()).toBeLessThan(1e-6);
  world.dispose();
});

it("cancels only one joint command when several joints act on a body", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  const moving = body();
  const first = new PrismaticJoint({ body0: null, body1: moving, axis: "X" });
  const second = new PrismaticJoint({ body0: null, body1: moving, axis: "X" });
  world.update(0);
  moving.applyForce(new Vector3(4, 0, 0));
  first.setEffort(20);
  second.setEffort(3);
  first.setEffort(0);
  world.update(world.fixedDelta);
  expect(moving.getVelocity().linear.x).toBeCloseTo(0.07);
  world.update(world.fixedDelta);
  expect(moving.getVelocity().linear.x).toBeCloseTo(0.07);
  world.dispose();
});
