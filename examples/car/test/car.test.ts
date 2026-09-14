import { expect, test } from "vitest";
import { Euler, Vector3 } from "three";
import { setupWorld } from "@drawcall/physics-rapier";
import { createCar, simulationOptions } from "../model";
import { driveCar } from "../drive";
import { createRoad } from "../road";

test("powered car crosses the bump course and brakes with independent suspension", async () => {
  const world = await setupWorld(simulationOptions);
  const car = createCar(world);
  createRoad(world);
  const driver = driveCar(world, car);
  const suspensions = car.wheels.map((wheel) => wheel.spring);
  let peakCompression = 0;
  let asymmetry = 0;
  let maxRearYaw = 0;
  let maxRoll = 0;
  let maxSpeed = 0;
  let minHeight = Infinity;
  for (let i = 0; i < 2400; i++) {
    world.update(world.fixedDelta);
    const compression = suspensions.map((joint) => joint.getState().position);
    peakCompression = Math.max(peakCompression, ...compression);
    const roll =
      (new Euler().setFromQuaternion(car.chassis.quaternion, "YXZ").z * 180) /
      Math.PI;
    for (const wheel of car.wheels.filter((wheel) => !wheel.front)) {
      // The axle direction ignores wheel spin and measures steering relative to the chassis.
      const axle = new Vector3(1, 0, 0)
        .applyQuaternion(wheel.tire.quaternion)
        .applyQuaternion(car.chassis.quaternion.clone().invert());
      maxRearYaw = Math.max(
        maxRearYaw,
        (Math.abs(Math.atan2(-axle.z, axle.x)) * 180) / Math.PI,
      );
    }
    expect(Number.isFinite(car.chassis.position.y)).toBe(true);
    maxSpeed = Math.max(maxSpeed, driver.telemetry.speed);
    maxRoll = Math.max(maxRoll, Math.abs(roll));
    minHeight = Math.min(minHeight, car.chassis.position.y);
    if (car.chassis.position.z > 28 && car.chassis.position.z < 38)
      asymmetry = Math.max(
        asymmetry,
        Math.abs((compression[0] ?? 0) - (compression[1] ?? 0)),
      );
  }
  expect(maxRearYaw).toBeLessThan(1);
  expect(driver.telemetry.completed).toBe(true);
  expect(car.chassis.position.z).toBeGreaterThan(59);
  expect(car.chassis.position.z).toBeLessThan(65);
  expect(Math.abs(driver.telemetry.speed)).toBeLessThan(0.2);
  expect(Math.abs(car.chassis.position.x)).toBeLessThan(0.8);
  expect(maxSpeed).toBeGreaterThan(4);
  expect(maxRoll).toBeLessThan(20);
  expect(minHeight).toBeGreaterThan(0.55);
  expect(asymmetry).toBeGreaterThan(0.025);
  expect(peakCompression).toBeGreaterThan(0.08);
  expect(peakCompression).toBeLessThan(0.24);
  driver.reset();
  expect(car.chassis.position.z).toBeCloseTo(0);
  driver.input.automatic = false;
  driver.input.throttle = -0.6;
  for (let i = 0; i < 120; i++) world.update(world.fixedDelta);
  expect(driver.telemetry.speed).toBeLessThan(-1);
  driver.input.throttle = 0;
  driver.input.brake = 1;
  for (let i = 0; i < 360; i++) world.update(world.fixedDelta);
  expect(Math.abs(driver.telemetry.speed)).toBeLessThan(0.2);
  driver.reset();
  driver.input.throttle = 0.4;
  driver.input.steer = 0.6;
  for (let i = 0; i < 360; i++) world.update(world.fixedDelta);
  expect(car.chassis.position.x).toBeGreaterThan(0.4);
  driver.dispose();
  world.dispose();
}, 60000);
