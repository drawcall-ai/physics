import { setupWorld } from "@drawcall/physics-rapier";
import { createCar, simulationOptions } from "./model";
import { createRoad } from "./road";
import { driveCar } from "./drive";
import { view } from "../view";

const world = await setupWorld(simulationOptions);
const car = createCar(world);
const road = createRoad(world);
road.add(car.root);
const driver = driveCar(world, car);
const demo = view(world, road, car.chassis.position);
const keys = new Set<string>();
window.addEventListener(
  "keydown",
  (event) => {
    if (event.code === "KeyR" || event.code === "KeyT") {
      keys.clear();
      if (event.code === "KeyT") driver.input.automatic = true;
      driver.reset();
    }
    if (!["KeyW", "KeyS", "KeyA", "KeyD", "Space"].includes(event.code)) return;
    event.preventDefault();
    demo.canvas.focus();
    keys.add(event.code);
    driver.input.automatic = false;
    input();
  },
  { signal: demo.signal },
);
window.addEventListener(
  "keyup",
  (event) => {
    keys.delete(event.code);
    input();
  },
  { signal: demo.signal },
);
window.addEventListener(
  "blur",
  () => {
    keys.clear();
    input();
  },
  { signal: demo.signal },
);
function input() {
  driver.input.throttle = Number(keys.has("KeyW")) - Number(keys.has("KeyS"));
  driver.input.steer = Number(keys.has("KeyA")) - Number(keys.has("KeyD"));
  driver.input.brake = Number(keys.has("Space"));
}
demo.run(() => {
  driver.updateVisuals();
  const { speed, completed } = driver.telemetry;
  return `${(speed * 3.6).toFixed(1)} km/h · ${driver.input.automatic ? (completed ? "Test complete" : "Automatic test") : "Manual"}`;
});
window.addEventListener(
  "pagehide",
  (event) => {
    if (event.persisted) return;
    driver.dispose();
    demo.dispose();
  },
  { signal: demo.signal },
);
