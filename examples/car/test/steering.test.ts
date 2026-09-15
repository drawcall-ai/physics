import { expect, test } from "vitest";
import { Vector3 } from "three";
import { RevoluteJoint } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import { createCar, simulationOptions, specification } from "../model";
import { createRoad } from "../road";
import { driveCar } from "../drive";

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
        for (const wheel of car.wheels.filter((wheel) => wheel.front)) {
          const axle = new Vector3(1, 0, 0)
            .applyQuaternion(wheel.tire.quaternion)
            .applyQuaternion(car.chassis.quaternion.clone().invert());
          const yaw = Math.atan2(-axle.z, axle.x);
          if (!(wheel.steering instanceof RevoluteJoint))
            throw new Error("Front wheel needs a steering joint");
          const steer =
            (direction * 0.45) / (1 + Math.abs(driver.telemetry.speed) * 0.06);
          const target =
            direction === 0
              ? 0
              : Math.atan(
                  specification.wheelbase /
                    (specification.wheelbase / Math.tan(steer) - wheel.x),
                );
          if (direction === 0) expect(target).toBe(0);
          else {
            expect(target * direction).toBeGreaterThan((20 * Math.PI) / 180);
            expect(target * direction).toBeLessThan((32 * Math.PI) / 180);
          }
          expect(Math.abs(yaw - target), wheel.name).toBeLessThan(
            Math.PI / 180,
          );
        }
      }
    } finally {
      driver.dispose();
      world.dispose();
    }
  },
);
