import { afterEach, describe, expect, test } from "vitest";
import { Scene, Vector3 } from "three";
import { BoxCollider, RigidBody, type PhysicsWorld } from "@drawcall/physics";
import { buildWorld as rapier } from "@drawcall/physics-rapier";
import { buildWorld as mujoco } from "@drawcall/physics-mujoco";
import { createCar } from "../car/model";
import { CarController } from "../car/controller";

const worlds: PhysicsWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

describe.each([
  { backend: "Rapier", build: rapier },
  { backend: "MuJoCo", build: mujoco },
])("$backend arcade car", ({ build }) => {
  async function setup(height = 0.85) {
    const car = createCar();
    car.body.position.y = height;
    const ground = new RigidBody({ bodyType: "static" });
    ground.position.y = -0.5;
    ground.add(new BoxCollider({ size: [500, 1, 500] }));
    const world = await build({
      scene: new Scene().add(car.body, ground),
      fixedDelta: 1 / 120,
    });
    worlds.push(world);
    const controller = new CarController(world, car);
    const advance = (seconds: number) => {
      for (let i = 0; i < Math.round(seconds / world.fixedDelta); i++)
        world.update(world.fixedDelta);
    };
    return { world, car, controller, advance };
  }

  test("suspension settles, accelerates, brakes, and reverses", async () => {
    const { car, controller, advance } = await setup();
    advance(2);
    expect(controller.contacts).toBe(4);
    expect(car.body.position.y).toBeGreaterThan(0.65);
    expect(car.body.position.y).toBeLessThan(0.85);
    expect(Math.abs(car.body.getVelocity().linear.y)).toBeLessThan(0.02);
    controller.input.throttle = 1;
    advance(3);
    expect(controller.speed).toBeGreaterThan(10);
    expect(car.body.position.z).toBeLessThan(-15);
    controller.input.throttle = 0;
    controller.input.brake = 1;
    advance(2);
    expect(Math.abs(controller.speed)).toBeLessThan(0.1);
    controller.input.brake = 0;
    controller.input.throttle = -1;
    advance(2);
    expect(controller.speed).toBeLessThan(-5);
  });

  test("steers right and stays upright through a sustained turn", async () => {
    const { car, controller, advance } = await setup();
    advance(1);
    controller.input.throttle = 0.7;
    advance(2);
    controller.input.steer = 0.8;
    advance(2);
    expect(car.body.position.x).toBeGreaterThan(3);
    const up = new Vector3(0, 1, 0).applyQuaternion(car.body.quaternion);
    expect(up.y).toBeGreaterThan(0.9);
    expect(controller.grounded).toBe(true);
  });

  test("throttle and steering do not create traction in the air", async () => {
    const { car, controller, advance } = await setup(8);
    controller.input.throttle = 1;
    controller.input.steer = 1;
    advance(0.5);
    expect(controller.grounded).toBe(false);
    expect(car.body.position.y).toBeLessThan(7);
    expect(Math.abs(car.body.position.x)).toBeLessThan(0.001);
    expect(Math.abs(car.body.position.z)).toBeLessThan(0.001);
    expect(car.body.getVelocity().angular.length()).toBeLessThan(0.001);
  });

  test("handbrake loosens the rear grip for a controllable slide", async () => {
    async function turn(handbrake: boolean) {
      const { car, controller, advance } = await setup();
      advance(1);
      controller.input.throttle = 1;
      advance(3);
      controller.input.throttle = 0.4;
      controller.input.steer = 0.8;
      controller.input.handbrake = handbrake;
      advance(0.8);
      const side = new Vector3(1, 0, 0).applyQuaternion(car.body.quaternion);
      const velocity = car.body.getVelocity().linear;
      const slip = Math.abs(velocity.dot(side)) / velocity.length();
      const up = new Vector3(0, 1, 0).applyQuaternion(car.body.quaternion);
      expect(up.y).toBeGreaterThan(0.9);
      return slip;
    }
    expect(await turn(true)).toBeGreaterThan((await turn(false)) + 0.1);
  });

  test("reset clears controls and dispose stops applying drive forces", async () => {
    const { world, car, controller, advance } = await setup();
    advance(1);
    controller.input.throttle =
      controller.input.steer =
      controller.input.brake =
        1;
    controller.input.handbrake = true;
    controller.reset();
    expect(controller.input).toEqual({
      throttle: 0,
      steer: 0,
      brake: 0,
      handbrake: false,
    });
    controller.dispose();
    world.reset();
    controller.input.throttle = 1;
    advance(0.25);
    expect(Math.abs(car.body.position.z)).toBeLessThan(0.001);
  });
});
