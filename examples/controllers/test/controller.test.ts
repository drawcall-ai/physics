import { afterEach, describe, expect, test } from "vitest";
import { Group, Matrix4, Quaternion, Scene, Vector3 } from "three";
import { BoxCollider, RigidBody, type PhysicsWorld } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import { CharacterController } from "../character/controller";
import { CapsuleMotor } from "../character/motor";

const worlds: PhysicsWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

function box(
  size: [number, number, number],
  x: number,
  y: number,
  z = 0,
  moving = false,
) {
  const body = new RigidBody({
    bodyType: moving ? "kinematic" : "static",
    colliders: false,
  });
  body.add(new BoxCollider({ size }));
  body.position.set(x, y, z);
  return body;
}

describe.each([
  { backend: "Rapier", build: buildRapier },
  { backend: "MuJoCo", build: buildMujoco },
])("$backend shared character", ({ build }) => {
  async function setup(
    obstacles: RigidBody[] = [],
    spawn = new Vector3(0, 0.02, 0),
    dimensions: { radius?: number; height?: number } = {},
  ) {
    const player = new Group();
    player.position.copy(spawn);
    const scene = new Scene().add(
      box([30, 0.2, 30], 0, -0.1),
      player,
      ...obstacles,
    );
    const world = await build({
      scene,
      gravity: [0, -20, 0],
      fixedDelta: 1 / 60,
    });
    worlds.push(world);
    const controller = new CharacterController(world, player, dimensions);
    const advance = (seconds: number) => {
      for (let i = 0; i < Math.round(seconds / world.fixedDelta); i++)
        world.update(world.fixedDelta);
    };
    advance(0.1);
    return { scene, player, world, controller, advance };
  }

  test("recovers from an initial floor overlap and reports unresolved penetration", async () => {
    const { player, controller, advance } = await setup(
      [],
      new Vector3(0, -0.1, 0),
    );
    expect(player.position.y).toBeGreaterThanOrEqual(0);
    expect(player.position.y).toBeLessThan(0.005);
    expect(controller.grounded).toBe(true);
    controller.input.crouching = true;
    advance(0.1);
    expect(controller.crouched).toBe(true);
    controller.input.crouching = false;
    advance(0.1);
    expect(controller.crouched).toBe(false);
    await expect(setup([box([10, 10, 10], 0, 0)])).rejects.toThrow(
      "Character capsule could not recover from overlapping geometry",
    );
  });

  test("recovers overlap without requiring extra space in a narrow corridor", async () => {
    const player = new Group();
    const scene = new Scene().add(
      box([1, 4, 4], -0.851, 1),
      box([1, 4, 4], 0.851, 1),
      player,
    );
    const world = await build({ scene });
    worlds.push(world);
    const motor = new CapsuleMotor(world, player, 0.35, 1.8);
    motor.recover();
    expect(motor.position.toArray()).toEqual([0, 0, 0]);

    motor.position.x = 0.002;
    motor.recover();
    expect(motor.position.x).toBeGreaterThan(-0.001);
    expect(motor.position.x).toBeLessThan(0.001);
  });

  test("crouching never enlarges a short capsule", async () => {
    const { player, controller, advance } = await setup(
      [],
      new Vector3(0, 0.02, 0),
      { radius: 0.35, height: 0.75 },
    );
    controller.input.crouching = true;
    advance(0.1);
    expect(controller.height).toBeLessThanOrEqual(0.75);
    expect(controller.height).toBeGreaterThan(0.7);
    expect(player.position.y).toBeCloseTo(0.004, 3);
    expect(controller.grounded).toBe(true);
    controller.input.crouching = false;
    advance(0.1);
    expect(controller.height).toBe(0.75);
  });

  test("stays grounded, stops at thin walls and slides along them", async () => {
    const { player, controller, advance } = await setup([
      box([0.03, 4, 12], 3, 2),
    ]);
    controller.input.direction.set(1, 0, 0.3).normalize();
    controller.input.running = true;
    advance(2);
    expect(player.position.x).toBeGreaterThan(2.5);
    expect(player.position.x).toBeLessThan(2.64);
    expect(player.position.z).toBeGreaterThan(2);
    expect(player.position.y).toBeCloseTo(0.004, 2);
    expect(controller.grounded).toBe(true);
  });

  test("jumps, lands, and resets without retaining momentum", async () => {
    const { player, controller, world, advance } = await setup();
    controller.jump();
    let peak = 0;
    for (let i = 0; i < 60; i++) {
      world.update(world.fixedDelta);
      peak = Math.max(peak, player.position.y);
      if (i === 3) expect(controller.grounded).toBe(false);
    }
    expect(peak).toBeGreaterThan(1.2);
    expect(peak).toBeLessThan(1.5);
    expect(controller.grounded).toBe(true);
    controller.input.direction.set(1, 0, 0);
    advance(0.2);
    controller.reset();
    advance(0.2);
    expect(player.position.x).toBeCloseTo(0, 4);
    expect(controller.speed).toBeLessThan(0.001);
  });

  test("climbs quarter-metre stairs and rejects taller ledges", async () => {
    const stairs = [0.25, 0.5, 0.75].map((height, index) =>
      box([0.8, height, 3], 1 + index * 0.8, height / 2),
    );
    const { player, controller, advance } = await setup(stairs);
    controller.input.direction.set(1, 0, 0);
    advance(0.7);
    expect(player.position.x).toBeGreaterThan(2.1);
    expect(player.position.y).toBeGreaterThan(0.7);
    expect(controller.grounded).toBe(true);
    for (const { height, z } of [
      { height: 0.34, z: 5 },
      { height: 0.7, z: 9 },
    ]) {
      controller.reset(new Vector3(-1, 0.01, z));
      player.parent?.add(box([1, height, 3], 1, height / 2, z));
      controller.input.direction.set(1, 0, 0);
      advance(1);
      expect(player.position.x).toBeLessThan(0.3);
      expect(player.position.y).toBeLessThan(0.1);
    }
  });

  test("walks up a shallow ramp but cannot walk up a steep ramp", async () => {
    const ramp = box([6, 0.2, 3], 0, 1.6);
    ramp.rotation.z = Math.PI / 6;
    const { player, controller, advance } = await setup(
      [ramp],
      new Vector3(-3.4, 0.02, 0),
    );
    controller.input.direction.set(1, 0, 0);
    advance(1.25);
    expect(player.position.x).toBeGreaterThan(-0.5);
    expect(player.position.y).toBeGreaterThan(1.3);
    expect(controller.grounded).toBe(true);
    const steep = box([4, 0.2, 3], 0, 1.84, 6);
    steep.rotation.z = Math.PI / 3;
    player.parent?.add(steep);
    controller.reset(new Vector3(-2, 0.02, 6));
    controller.input.direction.set(1, 0, 0);
    advance(1.5);
    expect(player.position.x).toBeLessThan(-0.9);
    expect(player.position.y).toBeLessThan(0.5);
  });

  test("crouches through a tunnel and refuses to stand into its ceiling", async () => {
    const roof = box([4, 0.3, 3], 2, 1.3);
    const { player, controller, advance } = await setup(
      [roof],
      new Vector3(-1, 0.02, 0),
    );
    controller.input.crouching = true;
    controller.input.direction.set(1, 0, 0);
    advance(1);
    expect(player.position.x).toBeGreaterThan(0.5);
    controller.input.direction.set(0, 0, 0);
    controller.input.crouching = false;
    advance(0.3);
    expect(controller.crouched).toBe(true);
    expect(player.position.y).toBeLessThan(0.02);
    controller.input.direction.set(1, 0, 0);
    advance(2);
    expect(player.position.x).toBeGreaterThan(4.4);
    expect(controller.crouched).toBe(false);
  });

  test("follows actual translating and rotating platform poses", async () => {
    const platform = box([5, 0.3, 5], 0, 0.45, 0, true);
    platform.name = "Platform";
    const { player, controller, world, advance } = await setup(
      [platform],
      new Vector3(1, 0.65, 0),
    );
    advance(0.4);
    const start = player.position
      .clone()
      .applyMatrix4(platform.matrixWorld.clone().invert());
    const pose = new Matrix4();
    const rotation = new Quaternion();
    const unit = new Vector3(1, 1, 1);
    const stopPlatform = world.onBeforeStep(() => {
      const time = world.time;
      platform.setKinematicTarget(
        pose.compose(
          new Vector3(time * 0.3, 0.45 + time * 0.2, 0),
          rotation.setFromAxisAngle(new Vector3(0, 1, 0), time * 0.4),
          unit,
        ),
      );
    });
    advance(2);
    const local = player.position
      .clone()
      .applyMatrix4(platform.matrixWorld.clone().invert());
    expect(local.distanceTo(start)).toBeLessThan(0.04);
    expect(controller.groundName).toBe("Platform");
    expect(player.position.y).toBeGreaterThan(0.9);
    stopPlatform();
    platform.removeFromParent();
    advance(0.6);
    expect(controller.groundName).not.toBe("Platform");
    expect(player.position.y).toBeLessThan(0.1);
  });
});
