import * as THREE from "three";
import { SoftBody } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { view } from "../view";
import { dragBars } from "./drag";
import {
  cargo,
  dampingRate,
  fabrics,
  hammock,
  simulationOptions,
  setDamping,
  type Cargo,
} from "./model";

function controls() {
  const fabric = document.querySelector<HTMLSelectElement>("#fabric");
  const debug = document.querySelector<HTMLInputElement>("#debug");
  const damping = document.querySelector<HTMLInputElement>("#damping");
  const reset = document.querySelector<HTMLButtonElement>("#reset");
  const backend = document.querySelector<HTMLButtonElement>("#backend");
  const output = document.querySelector("output");
  if (!fabric || !debug || !damping || !reset || !backend || !output)
    throw new Error("Missing hammock controls");
  return { fabric, debug, damping, reset, backend, output };
}

const controlsElement = controls();
let motionDamping = dampingRate;
controlsElement.damping.value = String(dampingRate);
const url = new URL(location.href);
const backend = url.searchParams.get("backend") ?? "rapier";
if (backend !== "rapier" && backend !== "mujoco")
  throw new Error(`Unknown physics backend: ${backend}`);
const backendName = backend === "mujoco" ? "MuJoCo" : "Rapier";
controlsElement.backend.textContent = `Switch to ${backend === "rapier" ? "MuJoCo" : "Rapier"}`;
controlsElement.backend.addEventListener("click", () => {
  url.searchParams.set("backend", backend === "rapier" ? "mujoco" : "rapier");
  location.assign(url);
});
const materials = fabrics(backend);
let failure: string | undefined;
window.addEventListener("unhandledrejection", (event) => {
  failure = String(event.reason);
  controlsElement.output.textContent = failure;
});
window.addEventListener("error", (event) => {
  failure = event.message;
  controlsElement.output.textContent = failure;
});
const model = hammock(backend);
const scene = new THREE.Scene().add(model.root);
const options = { scene, ...simulationOptions(backend) };
const world = await (backend === "mujoco"
  ? buildMujoco({ ...options, wasmUrl })
  : buildRapier(options));
const demo = view(world, scene, new THREE.Vector3(0, 1.6, 0));
demo.camera.position.set(5.4, 6.5, 6.5);
const release = dragBars(demo, world, model.bars);
const loads: ReturnType<typeof cargo>[] = [];
const nodes = new Map<SoftBody, THREE.Points>();
const nodeMaterial = new THREE.PointsMaterial({
  color: "#ffef9e",
  size: 0.035,
  depthTest: false,
});
let dropped = 0;

function wireframe(body: SoftBody, enabled: boolean) {
  body.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    const materials = Array.isArray(object.material)
      ? object.material
      : [object.material];
    for (const material of materials)
      if (material instanceof THREE.MeshStandardMaterial)
        material.wireframe = enabled;
  });
}

function showDebug() {
  const enabled = controlsElement.debug.checked;
  for (const body of [model.cloth, ...model.ropes, ...loads])
    if (body instanceof SoftBody) wireframe(body, enabled);
  for (const { marker } of model.anchors) marker.visible = enabled;
  for (const points of nodes.values()) points.visible = enabled;
}

function updateNodes() {
  if (!controlsElement.debug.checked) return;
  for (const body of [model.cloth, ...model.ropes, ...loads]) {
    if (!(body instanceof SoftBody)) continue;
    let points = nodes.get(body);
    if (!points) {
      points = new THREE.Points(new THREE.BufferGeometry(), nodeMaterial);
      points.frustumCulled = false;
      scene.add(points);
      nodes.set(body, points);
    }
    const positions = body.getPositions();
    const attribute = points.geometry.getAttribute("position");
    if (!attribute) {
      points.geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3),
      );
      continue;
    }
    attribute.array.set(positions);
    attribute.needsUpdate = true;
  }
}

for (const kind of ["rubber", "foam", "rigid"] satisfies Cargo[]) {
  const button = document.querySelector(`#${kind}`);
  if (!button) throw new Error(`Missing ${kind} control`);
  button.addEventListener("click", () => {
    const body = cargo(backend, kind, dropped++);
    setDamping(body, motionDamping);
    loads.push(body);
    scene.add(body);
    showDebug();
  });
}

function reset() {
  release();
  for (const body of loads) {
    if (body instanceof SoftBody) {
      const points = nodes.get(body);
      points?.removeFromParent();
      points?.geometry.dispose();
      nodes.delete(body);
    }
    body.removeFromParent();
    body.traverse((object) => {
      if (!(object instanceof THREE.Mesh)) return;
      object.geometry.dispose();
      const materials = Array.isArray(object.material)
        ? object.material
        : [object.material];
      for (const material of materials) material.dispose();
    });
  }
  loads.length = 0;
  dropped = 0;
  world.reset();
}
controlsElement.reset.addEventListener("click", reset);
controlsElement.debug.addEventListener("change", showDebug);
controlsElement.damping.addEventListener("input", () => {
  const rate = controlsElement.damping.valueAsNumber;
  const valid = Number.isFinite(rate) && rate >= 0;
  controlsElement.damping.setCustomValidity(
    valid ? "" : "Enter a finite, nonnegative damping rate.",
  );
  if (!valid) {
    controlsElement.damping.reportValidity();
    return;
  }
  motionDamping = rate;
  setDamping(scene, rate);
});
controlsElement.fabric.addEventListener("change", () => {
  const material = materials[controlsElement.fabric.value];
  if (!material) throw new Error("Unknown fabric profile");
  model.cloth.setMaterial(material);
});
window.addEventListener("keydown", (event) => {
  if (event.code === "KeyR") reset();
});
demo.run(() => {
  if (failure) return failure;
  updateNodes();
  for (const { marker, parent, point } of model.anchors)
    marker.position.copy(parent.localToWorld(point.clone()));
  return `${loads.length} cargo objects · ${backendName} · ${world.time.toFixed(1)} s`;
});
