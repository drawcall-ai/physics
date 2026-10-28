import { describe, expect, test } from "vitest";
import { Scene, Vector3 } from "three";
import {
  BoxCollider,
  RigidBody,
  type PhysicsWorld,
  type PhysicsWorldOptions,
} from "@drawcall/physics";
import { buildWorld as rapier } from "@drawcall/physics-rapier";
import { buildWorld as mujoco } from "@drawcall/physics-mujoco";
import { createPlane } from "../plane/model";
import { PlaneController } from "../plane/controller";
import { createQuadcopter } from "../quadcopter/model";
import { QuadController } from "../quadcopter/controller";

const backends = [
  { name: "Rapier", build: rapier },
  { name: "MuJoCo", build: mujoco },
];

function advance(world: PhysicsWorld, seconds: number) {
  for (let i = 0; i < Math.round(seconds / world.fixedDelta); i++)
    world.update(world.fixedDelta);
}

async function setup(
  body: RigidBody,
  build: (options: PhysicsWorldOptions) => Promise<PhysicsWorld>,
  fixedDelta = 1 / 120,
) {
  const floor = new RigidBody({ bodyType: "static", colliders: false });
  floor.position.y = -0.1;
  floor.add(new BoxCollider({ size: [1000, 0.2, 1000] }));
  const scene = new Scene().add(body, floor);
  return build({ scene, fixedDelta, maxSubsteps: 12 });
}

describe.each(backends)("$name flight", ({ build }) => {
  test("plane cruises level, climbs, banks into a turn and levels when released", async () => {
    const plane = createPlane();
    const world = await setup(plane.body, build);
    const controller = new PlaneController(world, plane);
    try {
      advance(world, 5);
      expect(plane.body.position.y).toBeCloseTo(8, 1);
      expect(plane.body.position.z).toBeLessThan(-85);
      expect(controller.speed).toBeGreaterThan(18);
      expect(controller.speed).toBeLessThan(21);
      controller.input.pitch = 0.7;
      controller.input.roll = 0.6;
      advance(world, 2);
      expect(plane.body.position.y).toBeGreaterThan(12);
      expect(
        new Vector3(0, 0, -1).applyQuaternion(plane.body.quaternion).x,
      ).toBeGreaterThan(0.5);
      expect(
        new Vector3(1, 0, 0).applyQuaternion(plane.body.quaternion).y,
      ).toBeLessThan(-0.25);
      controller.input.pitch = controller.input.roll = 0;
      advance(world, 3);
      expect(
        new Vector3(0, 1, 0).applyQuaternion(plane.body.quaternion).y,
      ).toBeGreaterThan(0.99);
      expect(Math.abs(plane.body.getVelocity().linear.y)).toBeLessThan(0.2);
      controller.input.boost = true;
      advance(world, 3);
      expect(controller.speed).toBeGreaterThan(29);
    } finally {
      controller.dispose();
      world.dispose();
    }
  });

  test("quad hovers, recovers from an impulse, moves and brakes without pose overrides", async () => {
    const quad = createQuadcopter();
    const world = await setup(quad.body, build);
    const controller = new QuadController(world, quad);
    try {
      advance(world, 3);
      expect(quad.body.position.y).toBeCloseTo(3, 2);
      quad.body.applyImpulse(new Vector3(3, 1, -2));
      expect(controller.speed).toBeGreaterThan(3);
      advance(world, 2);
      expect(controller.speed).toBeLessThan(0.01);
      expect(quad.body.position.x).toBeGreaterThan(0.6);
      controller.input.direction.set(1, 0, -1).normalize();
      controller.input.climb = 1;
      controller.input.yaw = 0.5;
      const before = quad.body.position.clone();
      advance(world, 2);
      expect(quad.body.position.x - before.x).toBeGreaterThan(7);
      expect(quad.body.position.y - before.y).toBeGreaterThan(6);
      expect(quad.body.position.z - before.z).toBeLessThan(-7);
      expect(
        new Vector3(0, 0, -1).applyQuaternion(quad.body.quaternion).x,
      ).toBeGreaterThan(0.8);
      controller.reset();
      advance(world, 2);
      expect(controller.speed).toBeLessThan(0.02);
      expect(
        new Vector3(0, 1, 0).applyQuaternion(quad.body.quaternion).y,
      ).toBeGreaterThan(0.99);
      controller.dispose();
      const height = quad.body.position.y;
      advance(world, 0.5);
      expect(quad.body.position.y).toBeLessThan(height - 1);
    } finally {
      controller.dispose();
      world.dispose();
    }
  });

  test("quad remains blocked by a wall and can move away again", async () => {
    const quad = createQuadcopter();
    const world = await setup(quad.body, build);
    const wall = new RigidBody({ bodyType: "static", colliders: false });
    wall.position.set(3, 3, 0);
    wall.add(new BoxCollider({ size: [0.3, 10, 20] }));
    world.scene.add(wall);
    const controller = new QuadController(world, quad);
    try {
      controller.input.direction.set(1, 0, 0);
      advance(world, 3);
      expect(quad.body.position.x).toBeLessThan(2.85);
      expect(quad.body.position.x).toBeGreaterThan(1);
      controller.input.direction.set(-1, 0, 0);
      advance(world, 1);
      expect(quad.body.position.x).toBeLessThan(0);
    } finally {
      controller.dispose();
      world.dispose();
    }
  });

  test("flight controls converge across fixed timesteps", async () => {
    const positions: Vector3[] = [];
    for (const fixedDelta of [1 / 60, 1 / 120]) {
      const quad = createQuadcopter();
      const world = await setup(quad.body, build, fixedDelta);
      const controller = new QuadController(world, quad);
      try {
        controller.input.direction.set(1, 0, -1).normalize();
        advance(world, 2);
        controller.reset();
        advance(world, 1);
        positions.push(quad.body.position.clone());
      } finally {
        controller.dispose();
        world.dispose();
      }
    }
    const [coarse, fine] = positions;
    if (!coarse || !fine) throw new Error("Missing timestep samples");
    expect(coarse.distanceTo(fine)).toBeLessThan(0.08);
  });
});
