import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export function createView(
  canvas: HTMLCanvasElement,
  scene: THREE.Scene,
  target: THREE.Object3D,
  offset: THREE.Vector3,
) {
  scene.background = new THREE.Color("#e8e9e2");
  scene.fog = new THREE.Fog("#e8e9e2", 90, 280);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.25;
  const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 500);
  camera.position.copy(target.position).add(offset);
  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(target.position).add(new THREE.Vector3(0, 1, -2));
  controls.enableDamping = true;
  controls.enablePan = false;
  controls.minDistance = 3;
  controls.maxDistance = 40;
  controls.maxPolarAngle = Math.PI * 0.48;
  controls.update();
  scene.add(new THREE.HemisphereLight("#fff9ed", "#6e7b73", 2.5));
  const sunlight = new THREE.DirectionalLight("#ffefd8", 3);
  sunlight.castShadow = true;
  sunlight.shadow.mapSize.set(2048, 2048);
  Object.assign(sunlight.shadow.camera, {
    left: -32,
    right: 32,
    top: 32,
    bottom: -32,
    near: 0.5,
    far: 100,
  });
  sunlight.shadow.normalBias = 0.025;
  scene.add(sunlight, sunlight.target);
  function resize() {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  }
  window.addEventListener("resize", resize);
  resize();
  const previous = target.position.clone();
  const movement = new THREE.Vector3();
  return {
    renderer,
    camera,
    render() {
      movement.copy(target.position).sub(previous);
      camera.position.add(movement);
      controls.target.add(movement);
      previous.copy(target.position);
      sunlight.position
        .copy(target.position)
        .add(new THREE.Vector3(-12, 35, 16));
      sunlight.target.position.copy(target.position);
      controls.update();
      renderer.render(scene, camera);
    },
    dispose() {
      window.removeEventListener("resize", resize);
      controls.dispose();
      const geometries = new Set<THREE.BufferGeometry>();
      const materials = new Set<THREE.Material>();
      const textures = new Set<THREE.Texture>();
      scene.traverse((object) => {
        if (!(object instanceof THREE.Mesh)) return;
        geometries.add(object.geometry);
        for (const material of Array.isArray(object.material)
          ? object.material
          : [object.material])
          materials.add(material);
      });
      for (const material of materials) {
        for (const value of Object.values(material))
          if (value instanceof THREE.Texture) textures.add(value);
        material.dispose();
      }
      for (const geometry of geometries) geometry.dispose();
      for (const texture of textures) texture.dispose();
      sunlight.dispose();
      renderer.dispose();
    },
  };
}
