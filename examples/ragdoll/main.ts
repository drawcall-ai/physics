import { ancestorBody } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { backend } from "../backend";
import { forwardHtmlEvents } from "@pmndrs/pointer-events";
import { createRagdoll, simulationOptions } from "./model";
import { grab } from "./grab";
import { view } from "../view";

const scene = createRagdoll();
const world = await (backend === "mujoco"
  ? buildMujoco({ ...simulationOptions, wasmUrl })
  : buildRapier(simulationOptions));
const demo = view(world, scene);
const pointer = forwardHtmlEvents(demo.canvas, demo.camera, scene, {
  batchEvents: false,
});
const grabs = new Map<number, ReturnType<typeof grab>>();
scene.addEventListener("pointerdown", (event) => {
  const body = ancestorBody(event.object);
  if (event.button !== 0 || body?.bodyType !== "dynamic") return;
  event.object.setPointerCapture(event.pointerId);
  grabs.set(event.pointerId, grab(scene, body, event.point));
  demo.controls.enabled = false;
});
scene.addEventListener("pointermove", (event) => {
  grabs.get(event.pointerId)?.move(event.point);
});
for (const type of ["pointerup", "pointercancel"] as const)
  scene.addEventListener(type, (event) => {
    grabs.get(event.pointerId)?.release();
    grabs.delete(event.pointerId);
    demo.controls.enabled = grabs.size === 0;
  });
window.addEventListener(
  "keydown",
  (event) => {
    if (event.code === "KeyR") world.reset();
  },
  { signal: demo.signal },
);
demo.run(() => {
  pointer.update();
  const held = [...grabs.values()].map((held) => held.body.name);
  return held.length ? `Holding ${held.join(", ")}` : "";
});
window.addEventListener(
  "pagehide",
  (event) => {
    if (event.persisted) return;
    pointer.destroy();
    demo.dispose();
  },
  { signal: demo.signal },
);
