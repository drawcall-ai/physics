import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { buildWorld } from "../backend";
import { CharacterController } from "./controller";
import { createCourse, simulationOptions } from "./model";
import "./style.css";

function elements() {
  const canvas = document.querySelector("canvas");
  const status = document.querySelector("output");
  const resetButton = document.querySelector<HTMLButtonElement>(".reset");
  if (!canvas || !status || !resetButton)
    throw new Error("Missing example controls");
  return { canvas, status, resetButton };
}
const { canvas, status, resetButton } = elements();
resetButton.disabled = true;

const scene = new THREE.Scene();
scene.background = new THREE.Color("#e8e9e2");
scene.fog = new THREE.Fog("#e8e9e2", 34, 85);
const course = createCourse();
const player = new THREE.Group();
player.name = "Character";
player.position.copy(course.spawn);
scene.add(course.root, player);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.25;
const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 120);
camera.position.set(9, 9, 18);
const controls = new OrbitControls(camera, canvas);
controls.target.copy(player.position).add(new THREE.Vector3(0, 1, -2));
controls.enableDamping = true;
controls.enablePan = false;
controls.minDistance = 3;
controls.maxDistance = 27;
controls.maxPolarAngle = Math.PI * 0.47;
controls.update();

scene.add(new THREE.HemisphereLight("#fff9ed", "#6e7b73", 2.5));
const sunlight = new THREE.DirectionalLight("#ffefd8", 3);
sunlight.position.set(-12, 24, 16);
sunlight.target.position.set(0, 0, -4);
sunlight.castShadow = true;
sunlight.shadow.mapSize.set(2048, 2048);
Object.assign(sunlight.shadow.camera, {
  left: -26,
  right: 26,
  top: 26,
  bottom: -26,
  near: 0.5,
  far: 75,
});
sunlight.shadow.normalBias = 0.025;
scene.add(sunlight, sunlight.target);
const grid = new THREE.GridHelper(32, 32, "#b0b8ad", "#c2c8bd");
grid.position.set(0, 0.003, -5);
scene.add(grid);

for (const label of course.labels) {
  const image = document.createElement("canvas");
  image.width = 768;
  image.height = 96;
  const context = image.getContext("2d");
  if (!context) throw new Error("Could not render course labels");
  context.fillStyle = label.color;
  context.font = "600 42px system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(label.text, image.width / 2, image.height / 2);
  const texture = new THREE.CanvasTexture(image);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(4.6, 0.575),
    new THREE.MeshBasicMaterial({
      map: texture,
      transparent: true,
      depthWrite: false,
    }),
  );
  mesh.position.copy(label.position);
  mesh.rotation.x = -Math.PI / 2;
  scene.add(mesh);
}

const radius = 0.35;
const shell = new THREE.Group();
player.add(shell);
const material = new THREE.MeshStandardMaterial({
  color: "#e46f35",
  roughness: 0.4,
});
const cylinder = new THREE.Mesh(
  new THREE.CylinderGeometry(radius, radius, 1, 32),
  material,
);
const lower = new THREE.Mesh(
  new THREE.SphereGeometry(radius, 32, 16),
  material,
);
const upper = new THREE.Mesh(lower.geometry, material);
const heading = new THREE.Mesh(
  new THREE.SphereGeometry(0.075, 16, 8),
  new THREE.MeshStandardMaterial({ color: "#fff5d9", roughness: 0.7 }),
);
for (const mesh of [cylinder, lower, upper, heading]) {
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  shell.add(mesh);
}
lower.position.y = radius;

function resize() {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
}
window.addEventListener("resize", resize);
resize();
renderer.render(scene, camera);

function stop(error: unknown) {
  renderer.setAnimationLoop(null);
  status.textContent = `Simulation stopped: ${String(error)}`;
  document.body.dataset.error = "true";
  console.error(error);
}

const world = await buildWorld({ scene, ...simulationOptions }).catch(
  (error: unknown) => {
    stop(error);
    throw error;
  },
);
world.onBeforeStep((delta) => course.update(world.time + delta));
const controller = new CharacterController(world, player, {
  radius,
  height: 1.8,
});
resetButton.disabled = false;
const keys = new Set<string>();
const movementKeys = new Set([
  "KeyW",
  "KeyA",
  "KeyS",
  "KeyD",
  "ArrowUp",
  "ArrowLeft",
  "ArrowDown",
  "ArrowRight",
  "Space",
  "ShiftLeft",
  "ShiftRight",
  "KeyC",
  "ControlLeft",
]);
const forward = new THREE.Vector3();
const right = new THREE.Vector3();
const up = new THREE.Vector3(0, 1, 0);
const previous = player.position.clone();
const movement = new THREE.Vector3();

function readInput() {
  camera.getWorldDirection(forward);
  forward.y = 0;
  forward.normalize();
  right.crossVectors(forward, up);
  const x =
    Number(keys.has("KeyD") || keys.has("ArrowRight")) -
    Number(keys.has("KeyA") || keys.has("ArrowLeft"));
  const z =
    Number(keys.has("KeyW") || keys.has("ArrowUp")) -
    Number(keys.has("KeyS") || keys.has("ArrowDown"));
  controller.input.direction
    .copy(forward)
    .multiplyScalar(z)
    .addScaledVector(right, x)
    .normalize();
  controller.input.running = keys.has("ShiftLeft") || keys.has("ShiftRight");
  controller.input.crouching = keys.has("KeyC") || keys.has("ControlLeft");
}

function press(code: string) {
  if (code === "Space" && !keys.has(code)) controller.jump();
  keys.add(code);
  readInput();
}

function release(code: string) {
  keys.delete(code);
  readInput();
}

function reset() {
  keys.clear();
  world.reset();
  course.reset();
  controller.reset(course.spawn);
  readInput();
  canvas.focus();
}

window.addEventListener("keydown", (event) => {
  if (
    event.target instanceof HTMLSelectElement ||
    event.target instanceof HTMLButtonElement
  )
    return;
  if (event.code === "KeyR") {
    event.preventDefault();
    reset();
    return;
  }
  if (!movementKeys.has(event.code)) return;
  event.preventDefault();
  press(event.code);
});
window.addEventListener("keyup", (event) => release(event.code));
window.addEventListener("blur", () => {
  keys.clear();
  readInput();
});
resetButton.addEventListener("click", reset);
canvas.addEventListener("pointerdown", () => canvas.focus());
for (const button of Array.from(
  document.querySelectorAll<HTMLButtonElement>("[data-key]"),
)) {
  const code = button.dataset.key;
  if (!code) throw new Error("Missing touch control binding");
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    press(code);
  });
  button.addEventListener("pointerup", () => release(code));
  button.addEventListener("pointercancel", () => release(code));
  button.addEventListener("lostpointercapture", () => release(code));
}

const timer = new THREE.Timer();
timer.connect(document);
renderer.setAnimationLoop(() => {
  timer.update();
  try {
    readInput();
    world.update(Math.min(timer.getDelta(), 0.1));
    if (player.position.y < -8) reset();
    const height = controller.height;
    cylinder.scale.y = Math.max(0.01, height - radius * 2);
    cylinder.position.y = height / 2;
    upper.position.y = height - radius;
    heading.position.set(0, height - radius - 0.08, -radius);
    if (controller.input.direction.lengthSq() > 0)
      shell.rotation.y = Math.atan2(
        -controller.input.direction.x,
        -controller.input.direction.z,
      );
    movement.copy(player.position).sub(previous);
    camera.position.add(movement);
    controls.target.add(movement);
    previous.copy(player.position);
    controls.update();
    status.textContent = `${controller.crouched ? "Crouching" : controller.grounded ? "Grounded" : "Airborne"} · ${controller.speed.toFixed(1)} m/s${controller.grounded ? ` · ${controller.groundName}` : ""}`;
    document.body.dataset.airborne = String(!controller.grounded);
    renderer.render(scene, camera);
  } catch (error) {
    stop(error);
  }
});

export const runtime = { controller, world, player, course, camera, renderer };

if (import.meta.hot)
  import.meta.hot.dispose(() => {
    renderer.setAnimationLoop(null);
    controls.dispose();
    timer.dispose();
    controller.dispose();
    world.dispose();
    renderer.dispose();
  });
