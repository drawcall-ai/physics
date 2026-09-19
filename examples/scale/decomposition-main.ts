import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { Vector3 } from "three";
import { backend } from "../backend";
import { view } from "../view";
import { decomposition } from "./decomposition";

const nav = document.querySelector("nav");
if (!nav) throw new Error("Missing navigation");
// Retain the backend selector installed by the shared example controls.
const controls = document.createElement("section");
controls.innerHTML = `
  <strong>Convex hull vs preserved opening</strong>
  <p>Two identical frame meshes. Two identical falling cubes.</p>
  <p><b>Left · gold · convex hull:</b> the cube stops above the opening.<br>
  <b>Right · teal · ${backend === "mujoco" ? "CoACD decomposition" : "triangle mesh"}:</b> the cube falls through to the floor.</p>
  <p>${backend === "mujoco" ? "MuJoCo decomposes the right frame into convex pieces when the world is built." : "Rapier collides the right frame's triangles directly. Select MuJoCo to verify convex decomposition."}</p>
  <button id="replay">Replay drops</button>
  <a href="?backend=${backend}">Back to scale checks</a>
  <p>P pause · drag to orbit</p>
  <output>Building colliders…</output>`;
for (const child of Array.from(nav.children)) {
  if (child.tagName !== "LABEL") child.remove();
}
nav.prepend(controls);
const { root, lanes } = decomposition();
const world = await (backend === "mujoco"
  ? buildMujoco({ wasmUrl })
  : buildRapier());
const demo = view(world, root, new Vector3(0, 1.5, 0));
document
  .querySelector("#replay")
  ?.addEventListener("click", () => world.reset(), { signal: demo.signal });
demo.run(() =>
  lanes
    .map(
      ({ cube }, index) =>
        `${index === 0 ? "Left" : "Right"} cube height: ${cube.position.y.toFixed(2)} m`,
    )
    .join(" · "),
);
window.addEventListener(
  "pagehide",
  (event) => {
    if (!event.persisted) demo.dispose();
  },
  { signal: demo.signal },
);
