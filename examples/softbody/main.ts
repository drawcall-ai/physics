import { PCFShadowMap, Vector3 } from "three";
import { buildWorld } from "@drawcall/physics-mujoco";
import { view } from "../view";
import { gripper, simulationOptions } from "./model.ts";

const grip = document.querySelector<HTMLInputElement>("#grip");
const lift = document.querySelector<HTMLInputElement>("#lift");
const reset = document.querySelector<HTMLButtonElement>("#reset");
const controls = document.querySelector<HTMLFieldSetElement>("#controls");
const select = document.querySelector<HTMLSelectElement>("#object");
const description = document.querySelector("#description");
const output = document.querySelector("output");
if (!grip || !lift || !reset || !controls || !select || !description || !output)
  throw new Error("Missing gripper controls");

window.addEventListener("unhandledrejection", (event) => {
  output.textContent = String(event.reason);
});
window.addEventListener("error", (event) => {
  output.textContent = event.message;
});
const url = new URL(location.href);
const kind = url.searchParams.get("object") ?? "cloth";
if (kind !== "block" && kind !== "cloth" && kind !== "rope")
  throw new Error(`Unknown object: ${kind}`);
select.value = kind;
select.addEventListener("change", () => {
  url.searchParams.set("object", select.value);
  location.assign(url);
});
description.textContent = {
  block: "Squeeze the block, lift it, then open to release.",
  cloth: "Pinch the cloth, lift it, then watch it drape.",
  rope: "Grip the cord, lift it, then let it swing.",
}[kind];
export const model = await gripper(`${import.meta.env.BASE_URL}pad.glts`, kind);
export const world = await buildWorld({
  scene: model.scene,
  ...simulationOptions,
});
const demo = view(world, model.scene, new Vector3(0, 0.1, 0));
demo.ambient.intensity = 1.4;
demo.light.intensity = 3.5;
demo.light.castShadow = true;
demo.light.shadow.mapSize.set(1024, 1024);
demo.light.shadow.normalBias = 0.0002;
demo.light.shadow.bias = -0.00003;
Object.assign(demo.light.shadow.camera, {
  left: -0.4,
  right: 0.4,
  top: 0.4,
  bottom: -0.4,
  near: 0.1,
  far: 50,
});
demo.light.shadow.camera.updateProjectionMatrix();
demo.renderer.shadowMap.enabled = true;
demo.renderer.shadowMap.type = PCFShadowMap;
demo.camera.position
  .set(0.35, 0.18, 0.45)
  .multiplyScalar(innerWidth < innerHeight ? 1.25 : 1)
  .add(demo.controls.target);
const target = { grip: 0, lift: 0 };
world.onBeforeStep(() => model.update(target, world.fixedDelta));
grip.addEventListener("input", () => (target.grip = grip.valueAsNumber / 100));
lift.addEventListener("input", () => (target.lift = lift.valueAsNumber / 100));
const status = () => {
  const height = Math.round(model.height() * 1000);
  const force = model.object
    .getContacts()
    .filter((contact) => model.pads.some((pad) => pad === contact.otherBody))
    .reduce((total, contact) => total + contact.force, 0);
  return `Height ${height} mm · Pad contact ${force.toFixed(2)} N`;
};
const restart = () => {
  target.grip = target.lift = 0;
  grip.value = lift.value = "0";
  model.reset();
  world.reset();
  demo.run(status);
};
reset.addEventListener("click", restart);
window.addEventListener("keydown", (event) => {
  if (event.code === "KeyR") restart();
});
controls.disabled = false;
demo.run(status);
