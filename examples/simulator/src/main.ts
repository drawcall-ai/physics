import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "@drawcall/physics-usd";
import { setupWorld } from "@drawcall/physics-rapier";
import { createRagdoll } from "./ragdoll";

const canvas = document.querySelector("canvas");
const status = document.querySelector("output");
const file = document.querySelector("input");
const pause = document.querySelector("#pause");
const reset = document.querySelector("#reset");
const record = document.querySelector("#record");
const exportButton = document.querySelector("#export");
if (
  !canvas ||
  !status ||
  !(file instanceof HTMLInputElement) ||
  !(pause instanceof HTMLButtonElement) ||
  !(reset instanceof HTMLButtonElement) ||
  !(record instanceof HTMLButtonElement) ||
  !(exportButton instanceof HTMLButtonElement)
) {
  throw new Error("Missing simulator controls");
}

async function start(
  canvas: HTMLCanvasElement,
  status: HTMLOutputElement,
  file: HTMLInputElement,
  pause: HTMLButtonElement,
  reset: HTMLButtonElement,
  record: HTMLButtonElement,
  exportButton: HTMLButtonElement,
) {
  let world = await setupWorld();
  let asset: THREE.Object3D = createRagdoll();
  let name = "ragdoll";
  let gravity: [number, number, number] = [0, -9.81, 0];
  let paused = false;
  let busy = false;
  let recorder: MediaRecorder | undefined;
  const scene = new THREE.Scene();
  scene.add(asset, new THREE.HemisphereLight(0xffffff, 0x667788, 3));
  const light = new THREE.DirectionalLight(0xffffff, 3);
  light.position.set(3, 5, 4);
  scene.add(light);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setClearColor(0x18212c);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 1000);
  const controls = new OrbitControls(camera, canvas);
  function frame() {
    const bounds = new THREE.Box3().setFromObject(asset);
    const center = bounds.getCenter(new THREE.Vector3());
    const radius = Math.max(
      bounds.getSize(new THREE.Vector3()).length() / 2,
      1,
    );
    controls.target.copy(center);
    camera.position
      .copy(center)
      .add(
        new THREE.Vector3(0.7, 0.6, 1).normalize().multiplyScalar(radius * 2),
      );
    camera.far = radius * 100;
    camera.updateProjectionMatrix();
    controls.update();
  }
  function resize() {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  }
  function disposeVisuals(root: THREE.Object3D) {
    root.traverse((object) => {
      if (!(object instanceof THREE.Mesh)) return;
      object.geometry.dispose();
      for (const material of Array.isArray(object.material)
        ? object.material
        : [object.material]) {
        for (const value of Object.values(material))
          if (value instanceof THREE.Texture) value.dispose();
        material.dispose();
      }
    });
    root.removeFromParent();
  }
  function download(blob: Blob, filename: string) {
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  function buttons() {
    file.disabled = busy || recorder !== undefined;
    reset.disabled = busy || recorder !== undefined;
    exportButton.disabled = busy || recorder !== undefined;
    pause.disabled = busy || recorder !== undefined;
    pause.textContent = paused ? "Play" : "Pause";
    record.disabled = busy || !recordingType;
    record.textContent = recorder ? "Stop & save video" : "Record video";
  }
  function fail(error: unknown) {
    paused = true;
    if (recorder?.state === "recording") recorder.stop();
    status.textContent = String(error);
    buttons();
  }
  const recordingType =
    typeof MediaRecorder === "undefined"
      ? undefined
      : ["video/webm;codecs=vp9", "video/webm;codecs=vp8", "video/webm"].find(
          (type) => MediaRecorder.isTypeSupported(type),
        );
  if (!recordingType)
    record.title = "This browser does not support WebM recording";
  pause.onclick = () => {
    paused = !paused;
    buttons();
  };
  reset.onclick = () => {
    try {
      world.reset();
    } catch (error) {
      fail(error);
    }
  };
  file.onchange = async () => {
    const input = file.files?.[0];
    if (!input) return;
    busy = true;
    buttons();
    status.textContent = `Opening ${input.name}…`;
    let nextWorld: typeof world | undefined;
    let nextAsset: THREE.Object3D | undefined;
    try {
      const bytes = await input.arrayBuffer();
      // Read authored gravity before creating the simulation world.
      const authored = await new PhysicsUSDLoader().parseAsync(bytes);
      const nextGravity = authored.gravity;
      disposeVisuals(authored);
      authored.dispose();
      nextWorld = await setupWorld({ gravity: nextGravity });
      nextAsset = await new PhysicsUSDLoader({ world: nextWorld }).parseAsync(
        bytes,
      );
      nextWorld.reset();
      disposeVisuals(asset);
      world.dispose();
      world = nextWorld;
      asset = nextAsset;
      gravity = nextGravity;
      name = input.name.replace(/\.[^.]+$/, "");
      scene.add(asset);
      frame();
      paused = false;
      status.textContent = `Playing ${input.name}`;
    } catch (error) {
      if (nextAsset) disposeVisuals(nextAsset);
      nextWorld?.dispose();
      fail(error);
    } finally {
      busy = false;
      file.value = "";
      buttons();
    }
  };
  exportButton.onclick = async () => {
    busy = true;
    buttons();
    try {
      world.reset();
      const bytes = await new PhysicsUSDExporter().parseAsync(asset, {
        gravity,
      });
      download(
        new Blob([bytes], { type: "model/vnd.usdz+zip" }),
        `${name}.usdz`,
      );
    } catch (error) {
      fail(error);
    } finally {
      busy = false;
      buttons();
    }
  };
  record.onclick = () => {
    if (recorder) {
      recorder.stop();
      return;
    }
    if (!recordingType) return;
    try {
      world.reset();
      paused = false;
      const stream = canvas.captureStream(30);
      const recording = new MediaRecorder(stream, { mimeType: recordingType });
      const chunks: Blob[] = [];
      recording.ondataavailable = (event) => {
        if (event.data.size) chunks.push(event.data);
      };
      recording.onstop = () => {
        for (const track of stream.getTracks()) track.stop();
        recorder = undefined;
        if (chunks.length)
          download(new Blob(chunks, { type: recordingType }), `${name}.webm`);
        status.textContent = "Video saved";
        buttons();
      };
      recording.onerror = () => fail(new Error("Video recording failed"));
      recording.start();
      recorder = recording;
      status.textContent = "Recording… click Stop & save video to download";
      buttons();
    } catch (error) {
      fail(error);
    }
  };
  resize();
  frame();
  buttons();
  status.textContent = "Playing ragdoll · open a USDZ or record a video";
  window.addEventListener("resize", resize);
  const timer = new THREE.Timer();
  timer.connect(document);
  renderer.setAnimationLoop(() => {
    timer.update();
    try {
      if (!busy && !paused) world.update(timer.getDelta());
    } catch (error) {
      fail(error);
    }
    renderer.render(scene, camera);
  });
  window.addEventListener(
    "pagehide",
    () => {
      if (recorder?.state === "recording") recorder.stop();
      renderer.setAnimationLoop(null);
      disposeVisuals(asset);
      world.dispose();
      controls.dispose();
      renderer.dispose();
      timer.dispose();
      window.removeEventListener("resize", resize);
    },
    { once: true },
  );
}
start(canvas, status, file, pause, reset, record, exportButton).catch(
  (error) => {
    status.textContent = String(error);
    console.error(error);
  },
);
