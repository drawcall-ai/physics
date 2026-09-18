import { describe, expect, test } from "vitest";
import { Vector3 } from "three";
import { RigidBody, BoxCollider } from "@drawcall/physics";
import { setupWorld as setupRapier } from "@drawcall/physics-rapier";
import { setupWorld as setupMujoco } from "@drawcall/physics-mujoco";
import { createCar, simulationOptions } from "../model";
import { createRoad } from "../road";
import { driveCar } from "../drive";

describe.each([
  { backend: "Rapier", setupWorld: setupRapier },
  { backend: "MuJoCo", setupWorld: setupMujoco },
])("$backend steering", ({ setupWorld }) => {
  test.each([
    { brake: 0, fixedDelta: 1 / 120 },
    { brake: 1, fixedDelta: 1 / 120 },
    { brake: 0, fixedDelta: 1 / 240 },
    { brake: 1, fixedDelta: 1 / 240 },
  ])(
    "both front wheels steer and recenter at rest with brake=$brake and dt=$fixedDelta",
    async ({ brake, fixedDelta }) => {
      const world = await setupWorld({ ...simulationOptions, fixedDelta });
      const car = createCar(world);
      createRoad(world);
      const driver = driveCar(world, car);
      driver.input.automatic = false;
      driver.input.brake = brake;
      try {
        for (let i = 0; i < Math.round(2 / fixedDelta); i++)
          world.update(world.fixedDelta);
        for (const direction of [1, -1, 0]) {
          driver.input.steer = direction;
          for (let i = 0; i < Math.round(3 / fixedDelta); i++)
            world.update(world.fixedDelta);
          expect(Math.abs(driver.telemetry.speed)).toBeLessThan(
            brake ? 0.1 : 0.3,
          );
          // Sample every step so an oscillating servo cannot pass by crossing its target.
          for (let i = 0; i < Math.round(0.25 / fixedDelta); i++) {
            world.update(world.fixedDelta);
            for (const wheel of car.wheels.filter((wheel) => wheel.front)) {
              const axle = new Vector3(1, 0, 0)
                .applyQuaternion(wheel.tire.quaternion)
                .applyQuaternion(car.chassis.quaternion.clone().invert());
              const yaw = Math.atan2(-axle.z, axle.x);
              const target = wheel.servo?.target?.position;
              if (target === undefined)
                throw new Error("Missing steering target");
              if (direction === 0) expect(target).toBe(0);
              else {
                expect(target * direction).toBeGreaterThan(
                  (20 * Math.PI) / 180,
                );
                expect(target * direction).toBeLessThan((32 * Math.PI) / 180);
              }
              expect(Math.abs(yaw - target), wheel.name).toBeLessThan(
                Math.PI / 180,
              );
            }
          }
        }
      } finally {
        driver.dispose();
        world.dispose();
      }
    },
  );
  test.each([
    { throttle: 0, brake: 0 },
    { throttle: 0, brake: 1 },
    { throttle: 0.4, brake: 0 },
    { throttle: 1, brake: 0 },
    { throttle: -1, brake: 0 },
  ])(
    "tracks steering reversals without wobble with throttle=$throttle and brake=$brake",
    async ({ throttle, brake }) => {
      const world = await setupWorld(simulationOptions);
      const car = createCar(world);
      const floor = new RigidBody({ world, type: "static" });
      floor.position.y = -0.15;
      const collider = new BoxCollider({ size: [200, 0.3, 200] });
      collider.setCollisionGroups({ membership: 1, filter: 2 });
      floor.add(collider);
      const driver = driveCar(world, car);
      driver.input.automatic = false;
      driver.input.brake = brake;
      const front = car.wheels.filter((wheel) => wheel.front);
      const previous = front.map(() => 0);
      try {
        for (let step = 0; step < 1200; step++) {
          driver.input.throttle = step < 240 ? 0 : throttle;
          driver.input.steer =
            step < 300
              ? 0
              : step < 480
                ? 1
                : step < 660
                  ? -1
                  : step < 840
                    ? 0
                    : 1;
          world.update(world.fixedDelta);
          for (const [index, wheel] of front.entries()) {
            const axle = new Vector3(1, 0, 0)
              .applyQuaternion(wheel.tire.quaternion)
              .applyQuaternion(car.chassis.quaternion.clone().invert());
            const yaw = Math.atan2(-axle.z, axle.x);
            const target = wheel.servo?.target?.position;
            const last = previous[index];
            if (target === undefined || last === undefined)
              throw new Error("Missing steering state");
            const context = `${wheel.name} at step ${step}`;
            expect(Math.abs(yaw - target), context).toBeLessThan(
              (10 * Math.PI) / 180,
            );
            expect(Math.abs(yaw - last), context).toBeLessThan(
              (5 * Math.PI) / 180,
            );
            previous[index] = yaw;
          }
        }
      } finally {
        driver.dispose();
        world.dispose();
      }
    },
  );
});
