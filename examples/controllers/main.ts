import { Scene, Timer, Vector3 } from "three";
import { buildWorld } from "../backend";
import { createCharacterDemo } from "./character/demo";
import { createCarDemo, createPlaneDemo, createQuadDemo } from "./vehicles";
import { bindInput } from "./input";
import { createView } from "./view";
import "./style.css";

function elements() {
  const canvas = document.querySelector("canvas");
  const status = document.querySelector("output");
  const resetButton = document.querySelector<HTMLButtonElement>(".reset");
  const description = document.querySelector(".description");
  const hints = document.querySelector(".hints");
  const actions = document.querySelector(".actions");
  const modes = document.querySelector(".modes");
  if (
    !canvas ||
    !status ||
    !resetButton ||
    !description ||
    !hints ||
    !actions ||
    !modes
  )
    throw new Error("Missing playground controls");
  return { canvas, status, resetButton, description, hints, actions, modes };
}
const { canvas, status, resetButton, description, hints, actions, modes } =
  elements();
resetButton.disabled = true;
function report(error: unknown) {
  status.textContent = `Simulation stopped: ${String(error)}`;
  document.body.dataset.error = "true";
  resetButton.disabled = true;
  console.error(error);
}
async function start() {
  const url = new URL(location.href);
  const kind = url.searchParams.get("controller") ?? "character";
  function createDemo() {
    switch (kind) {
      case "character":
        return createCharacterDemo();
      case "car":
        return createCarDemo();
      case "plane":
        return createPlaneDemo();
      case "quadcopter":
        return createQuadDemo();
      default:
        throw new Error(`Unknown controller: ${kind}`);
    }
  }
  for (const [value, label] of Object.entries({
    character: "Character",
    car: "Car",
    plane: "Plane",
    quadcopter: "Quadcopter",
  })) {
    const link = document.createElement("a");
    const destination = new URL(url);
    destination.searchParams.set("controller", value);
    link.href = destination.href;
    link.textContent = label;
    if (value === kind) link.setAttribute("aria-current", "page");
    modes.append(link);
  }
  const demo = createDemo();
  description.textContent = demo.description;
  hints.textContent =
    kind === "plane"
      ? "W/S climb/dive · A/D bank"
      : kind === "car"
        ? "W/S drive/reverse · A/D steer"
        : "WASD / arrows move";
  const keyNames: Record<string, string> = {
    Space: "Space",
    ShiftLeft: "Shift",
    KeyC: "C",
    KeyQ: "Q",
    KeyE: "E",
  };
  for (const { code, label } of demo.keys) {
    const name = keyNames[code];
    if (!name) throw new Error(`Unknown control label: ${code}`);
    const hint = document.createElement("span");
    hint.textContent = `${name} ${label.toLowerCase()}`;
    hints.append(hint);
    const button = document.createElement("button");
    button.dataset.key = code;
    button.textContent = label;
    actions.append(button);
  }
  const scene = new Scene().add(demo.root);
  const view = createView(canvas, scene, demo.target, demo.camera);
  view.render();
  const world = await buildWorld({ scene, ...demo.options });
  const driver = demo.start(world);
  const input = bindInput(canvas, (code) => driver.press(code), reset);
  const forward = new Vector3();
  function reset() {
    input.clear();
    world.reset();
    driver.reset();
    canvas.focus();
  }
  resetButton.disabled = false;
  resetButton.addEventListener("click", reset);
  const timer = new Timer();
  timer.connect(document);
  view.renderer.setAnimationLoop(() => {
    timer.update();
    try {
      view.camera.getWorldDirection(forward);
      forward.y = 0;
      forward.normalize();
      driver.readInput(input.keys, forward);
      world.update(Math.min(timer.getDelta(), 0.1));
      if (demo.target.position.y < -8) reset();
      status.textContent = driver.update();
      view.render();
    } catch (error) {
      view.renderer.setAnimationLoop(null);
      report(error);
    }
  });
  const runtime = {
    world,
    demo,
    driver,
    camera: view.camera,
    renderer: view.renderer,
  };
  if (import.meta.hot)
    import.meta.hot.dispose(() => {
      view.renderer.setAnimationLoop(null);
      resetButton.removeEventListener("click", reset);
      timer.dispose();
      input.dispose();
      driver.dispose();
      world.dispose();
      view.dispose();
    });

  return runtime;
}
export const runtime = await start().catch((error: unknown) => {
  report(error);
  throw error;
});
