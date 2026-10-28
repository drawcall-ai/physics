import { Vector3 } from "three";
import { createCar } from "./car/model";
import { CarController } from "./car/controller";
import { createPlane } from "./plane/model";
import { PlaneController } from "./plane/controller";
import { createQuadcopter } from "./quadcopter/model";
import { QuadController } from "./quadcopter/controller";
import { createField } from "./field";
import { direction, held, type Demo } from "./demo";

const options = { fixedDelta: 1 / 120, maxSubsteps: 12 };

export function createCarDemo(): Demo {
  const root = createField(false);
  const car = createCar();
  car.body.position.z = 10;
  root.add(car.body);
  return {
    root,
    target: car.body,
    options,
    description: "Weave through the markers, drift, and hit the jump ramp.",
    keys: [
      { code: "Space", label: "Brake" },
      { code: "ShiftLeft", label: "Drift" },
    ],
    camera: new Vector3(6, 6, 11),
    start(world) {
      const controller = new CarController(world, car);
      return {
        readInput(keys) {
          const { x, z } = direction(keys);
          controller.input.throttle = z;
          controller.input.steer = x;
          controller.input.brake = Number(keys.has("Space"));
          controller.input.handbrake = held(keys, "ShiftLeft", "ShiftRight");
        },
        press() {},
        update: () =>
          `${controller.grounded ? "Driving" : "Airborne"} · ${Math.abs(controller.speed * 3.6).toFixed(0)} km/h`,
        reset: () => controller.reset(),
        dispose: () => controller.dispose(),
      };
    },
  };
}

export function createPlaneDemo(): Demo {
  const root = createField(true);
  const plane = createPlane();
  root.add(plane.body);
  return {
    root,
    target: plane.body,
    options,
    description: "Bank to turn, climb through the rings. Release to level out.",
    keys: [
      { code: "ShiftLeft", label: "Boost" },
      { code: "Space", label: "Slow" },
    ],
    camera: new Vector3(3, 4, 13),
    start(world) {
      const controller = new PlaneController(world, plane);
      return {
        readInput(keys) {
          const { x, z } = direction(keys);
          controller.input.pitch = z;
          controller.input.roll = x;
          controller.input.throttle = keys.has("Space") ? 0.15 : 0.55;
          controller.input.boost = held(keys, "ShiftLeft", "ShiftRight");
        },
        press() {},
        update() {
          plane.propeller.rotation.z = world.time * 45;
          return `Flying · ${controller.speed.toFixed(1)} m/s · ${plane.body.position.y.toFixed(1)} m altitude`;
        },
        reset: () => controller.reset(),
        dispose: () => controller.dispose(),
      };
    },
  };
}

export function createQuadDemo(): Demo {
  const root = createField(true);
  const quad = createQuadcopter();
  root.add(quad.body);
  return {
    root,
    target: quad.body,
    options,
    description:
      "Fly in any direction. Release to hover; land on the gold pad.",
    keys: [
      { code: "Space", label: "Up" },
      { code: "KeyC", label: "Down" },
      { code: "ShiftLeft", label: "Boost" },
      { code: "KeyQ", label: "Yaw left" },
      { code: "KeyE", label: "Yaw right" },
    ],
    camera: new Vector3(6, 5, 10),
    start(world) {
      const controller = new QuadController(world, quad);
      return {
        readInput(keys, forward) {
          const { x, z } = direction(keys);
          const right = new Vector3(-forward.z, 0, forward.x);
          controller.input.direction
            .copy(forward)
            .multiplyScalar(z)
            .addScaledVector(right, x)
            .normalize();
          controller.input.climb =
            Number(keys.has("Space")) -
            Number(held(keys, "KeyC", "ControlLeft"));
          controller.input.yaw =
            Number(keys.has("KeyE")) - Number(keys.has("KeyQ"));
          controller.input.boost = held(keys, "ShiftLeft", "ShiftRight");
        },
        press() {},
        update() {
          for (const [i, rotor] of quad.rotors.entries())
            rotor.rotation.y = world.time * 60 * (i % 2 === 0 ? 1 : -1);
          return `Hover assist · ${controller.speed.toFixed(1)} m/s · ${quad.body.position.y.toFixed(1)} m altitude`;
        },
        reset: () => controller.reset(),
        dispose: () => controller.dispose(),
      };
    },
  };
}
