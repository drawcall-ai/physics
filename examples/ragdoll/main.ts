import { setupWorld } from "@drawcall/physics-rapier";
import { createRagdoll, simulationOptions } from "./model";
import { view } from "../view";

const world = await setupWorld(simulationOptions);
const demo = view(world, createRagdoll());
window.addEventListener(
  "keydown",
  (event) => {
    if (event.code === "KeyR") world.reset();
  },
  { signal: demo.signal },
);
demo.run();
window.addEventListener(
  "pagehide",
  (event) => {
    if (!event.persisted) demo.dispose();
  },
  { signal: demo.signal },
);
