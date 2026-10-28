import * as THREE from "three";
import { CharacterController } from "./controller";
import { createCourse, simulationOptions } from "./course";
import { direction, held, type Demo } from "../demo";

export function createCharacterDemo(): Demo {
  const course = createCourse();
  const player = new THREE.Group();
  player.name = "Character";
  player.position.copy(course.spawn);
  course.root.add(player);
  const shell = new THREE.Group();
  player.add(shell);
  const radius = 0.35;
  const material = new THREE.MeshStandardMaterial({
    color: "#e46f35",
    roughness: 0.4,
  });
  const cylinder = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, 1, 24),
    material,
  );
  const lower = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 24, 12),
    material,
  );
  const upper = new THREE.Mesh(lower.geometry, material);
  const heading = new THREE.Mesh(
    new THREE.SphereGeometry(0.075, 12, 8),
    new THREE.MeshStandardMaterial({ color: "#fff5d9" }),
  );
  for (const mesh of [cylinder, lower, upper, heading]) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    shell.add(mesh);
  }
  lower.position.y = radius;
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
    context.fillText(label.text, 384, 48);
    const texture = new THREE.CanvasTexture(image);
    texture.colorSpace = THREE.SRGBColorSpace;
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
    course.root.add(mesh);
  }
  return {
    root: course.root,
    target: player,
    description: "Walk, jump, crouch. Try the stairs and moving platforms.",
    keys: [
      { code: "Space", label: "Jump" },
      { code: "KeyC", label: "Crouch" },
      { code: "ShiftLeft", label: "Run" },
    ],
    camera: new THREE.Vector3(9, 9, 15),
    options: simulationOptions,
    start(world) {
      const controller = new CharacterController(world, player, {
        radius,
        height: 1.8,
      });
      const unsubscribe = world.onBeforeStep((dt) =>
        course.update(world.time + dt),
      );
      return {
        readInput(keys, forward) {
          const { x, z } = direction(keys);
          const right = new THREE.Vector3(-forward.z, 0, forward.x);
          controller.input.direction
            .copy(forward)
            .multiplyScalar(z)
            .addScaledVector(right, x)
            .normalize();
          controller.input.running = held(keys, "ShiftLeft", "ShiftRight");
          controller.input.crouching = held(keys, "KeyC", "ControlLeft");
        },
        press(code) {
          if (code === "Space") controller.jump();
        },
        update() {
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
          return `${controller.crouched ? "Crouching" : controller.grounded ? "Grounded" : "Airborne"} · ${controller.speed.toFixed(1)} m/s`;
        },
        reset() {
          course.reset();
          controller.reset(course.spawn);
        },
        dispose() {
          unsubscribe();
          controller.dispose();
        },
      };
    },
  };
}
