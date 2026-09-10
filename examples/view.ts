import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import "./style.css";

export function view(
  world: { update(delta: number): void; dispose(): void },
  root: THREE.Object3D,
  target = new THREE.Vector3(0, 1, 0),
) {
  const canvas = document.querySelector("canvas");
  const status = document.querySelector("output");
  if (!canvas || !status) throw new Error("Missing canvas or status output");
  const scene = new THREE.Scene();
  scene.add(root, new THREE.HemisphereLight(0xffffff, 0x667788, 3));
  const light = new THREE.DirectionalLight(0xffffff, 3);
  scene.add(light, light.target);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setClearColor(0x18212c);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 200);
  camera.position.copy(target).add(new THREE.Vector3(8, 5, 9));
  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(target);
  const previous = target.clone();
  const timer = new THREE.Timer();
  timer.connect(document);
  const events = new AbortController();
  let paused = false;
  function resize() {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  }
  window.addEventListener("resize", resize, { signal: events.signal });
  window.addEventListener(
    "keydown",
    (event) => {
      if (event.code === "KeyP") paused = !paused;
    },
    { signal: events.signal },
  );
  resize();
  return {
    canvas,
    signal: events.signal,
    run(update: () => string = () => "") {
      renderer.setAnimationLoop(() => {
        timer.update();
        try {
          if (!paused) world.update(Math.min(timer.getDelta(), 0.1));
          status.textContent = update();
          if (paused) status.textContent = "Paused";
          const movement = target.clone().sub(previous);
          camera.position.add(movement);
          controls.target.add(movement);
          previous.copy(target);
          light.position.copy(target).add(new THREE.Vector3(-12, 24, 14));
          light.target.position.copy(target);
          controls.update();
          renderer.render(scene, camera);
        } catch (error) {
          renderer.setAnimationLoop(null);
          status.textContent = String(error);
          console.error(error);
        }
      });
    },
    dispose() {
      events.abort();
      renderer.setAnimationLoop(null);
      world.dispose();
      controls.dispose();
      timer.dispose();
      scene.traverse((object) => {
        if (!(object instanceof THREE.Mesh)) return;
        object.geometry.dispose();
        for (const material of Array.isArray(object.material)
          ? object.material
          : [object.material])
          material.dispose();
      });
      renderer.dispose();
    },
  };
}
