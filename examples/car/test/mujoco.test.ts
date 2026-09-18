import { expect, test } from "vitest";
import { setupWorld } from "@drawcall/physics-mujoco";
import { createCar, simulationOptions } from "../model";
import { createRoad } from "../road";
import { createGoal } from "../goal";
import { driveCar } from "../drive";

test("MuJoCo drives, steers, resets, and brakes the suspension car", async () => {
  const world = await setupWorld(simulationOptions);
  try {
    const car = createCar(world);
    createRoad(world);
    const goal = createGoal(car.chassis);
    const driver = driveCar(world, car);
    for (let i = 0; i < 2400; i++) world.update(world.fixedDelta);
    expect(driver.telemetry.completed).toBe(true);
    expect(goal.trigger.overlaps(car.chassis)).toBe(true);
    expect(Math.abs(driver.telemetry.speed)).toBeLessThan(0.2);
    driver.reset();
    expect(car.chassis.position.z).toBe(0);
    driver.input.automatic = false;
    driver.input.throttle = 0.4;
    driver.input.steer = 0.6;
    for (let i = 0; i < 360; i++) world.update(world.fixedDelta);
    expect(car.chassis.position.x).toBeGreaterThan(0.4);
    driver.dispose();
  } finally {
    world.dispose();
  }
}, 60000);
