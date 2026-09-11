import * as THREE from "three";
import { setupWorld } from "@drawcall/physics-rapier";
import { view } from "../view";
import { cases } from "./cases";
import { specimen, verify } from "./specimen";

const world = await setupWorld();
const root = new THREE.Group();
function controls() {
  const select = document.querySelector("select");
  const results = document.querySelector("#results");
  if (!select || !results) throw new Error("Missing example controls");
  return { select, results };
}
const { select, results } = controls();
for (const [index, spec] of cases.entries())
  select.add(new Option(spec.name, String(index)));
select.selectedIndex = cases.findIndex((spec) => spec.compound);
let current: ReturnType<typeof specimen> | undefined;
let checking = false;
let status = "";
let added = 0;
let removed = 0;
function remove() {
  if (!current) return;
  current.dispose();
  current = undefined;
  removed++;
}
function show() {
  if (checking) return;
  remove();
  const spec = cases[select.selectedIndex];
  if (!spec) throw new Error("Missing case");
  if (spec.error) {
    status = verify(world, spec);
    return;
  }
  current = specimen(world, spec, true);
  root.add(current.root);
  added++;
  status =
    spec.type === "kinematic"
      ? "Moving kinematic lift · teal cubes ride its collider"
      : "Spinning drop · R removes and drops it again";
}
const demo = view(
  {
    update(delta) {
      if (checking) return;
      world.update(delta);
    },
    dispose() {
      remove();
      world.dispose();
    },
  },
  root,
  new THREE.Vector3(0, 2, 0),
);
async function checkAll() {
  if (checking) return;
  remove();
  checking = true;
  select.disabled = true;
  results.textContent = "";
  let passed = 0;
  try {
    for (const [index, spec] of cases.entries()) {
      status = `Checking ${index + 1}/${cases.length}: ${spec.name}`;
      await new Promise(requestAnimationFrame);
      const row = document.createElement("li");
      try {
        // The second run creates fresh objects in the already-running world.
        verify(world, spec);
        const result = verify(world, spec);
        row.textContent = `${spec.name} — ${result}; removal/recreation PASS`;
        passed++;
      } catch (error) {
        row.textContent = `${spec.name} — FAIL: ${String(error)}`;
        row.style.color = "#ff9999";
      }
      results.append(row);
    }
    const summary = document.querySelector("summary");
    if (summary)
      summary.textContent = `${passed}/${cases.length} cases passed, including removal/recreation (expand results)`;
  } finally {
    checking = false;
    select.disabled = false;
  }
  show();
}
select.addEventListener("change", show, { signal: demo.signal });
for (const [id, index] of [
  ["compound", cases.findIndex((spec) => spec.compound)],
  [
    "kinematic",
    cases.findIndex((spec) => spec.type === "kinematic" && !spec.error),
  ],
] as const) {
  document.getElementById(id)?.addEventListener(
    "click",
    () => {
      if (checking) return;
      select.selectedIndex = index;
      show();
    },
    { signal: demo.signal },
  );
}
document
  .querySelector("#check")
  ?.addEventListener("click", () => void checkAll(), { signal: demo.signal });
document
  .querySelector("#recreate")
  ?.addEventListener("click", show, { signal: demo.signal });
window.addEventListener(
  "keydown",
  (event) => {
    if (checking) return;
    if (event.code === "KeyX") {
      remove();
      status = "Removed. R recreates this case.";
    }
    if (event.code === "KeyR") show();
    if (event.code === "KeyN") {
      select.selectedIndex = (select.selectedIndex + 1) % cases.length;
      show();
    }
  },
  { signal: demo.signal },
);
demo.run(() => `${status} · ${added} added / ${removed} removed`);
void checkAll();
window.addEventListener(
  "pagehide",
  (event) => {
    if (!event.persisted) demo.dispose();
  },
  { signal: demo.signal },
);
