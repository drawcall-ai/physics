import * as THREE from "three";
import { BoxCollider, RigidBody } from "@drawcall/physics";

export const simulationOptions = {
  fixedDelta: 1 / 60,
  maxSubsteps: 5,
  gravity: [0, -20, 0],
} as const;

export function createCourse() {
  const root = new THREE.Group();
  root.name = "Character course";
  const spawn = new THREE.Vector3(0, 0.05, 8);
  const labels: { text: string; position: THREE.Vector3; color: string }[] = [];
  const palette = {
    floor: "#d7d9cf",
    steps: "#c57550",
    slope: "#88a18c",
    steep: "#7d8e93",
    tunnel: "#a09cb1",
    walls: "#d1b77c",
    platform: "#729ea7",
  };
  const materials = new Map<string, THREE.MeshStandardMaterial>();

  function box(
    name: string,
    size: [number, number, number],
    position: [number, number, number],
    color: string,
    moving = false,
  ) {
    let material = materials.get(color);
    if (!material) {
      material = new THREE.MeshStandardMaterial({ color, roughness: 0.9 });
      materials.set(color, material);
    }
    const body = new RigidBody({
      bodyType: moving ? "kinematic" : "static",
      colliders: false,
    });
    body.name = name;
    body.position.set(...position);
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    body.add(new BoxCollider({ size }), mesh);
    root.add(body);
    return body;
  }

  function label(text: string, x: number, z: number, color = "#3c4846") {
    labels.push({ text, position: new THREE.Vector3(x, 0.012, z), color });
  }

  box("Ground", [32, 0.3, 34], [0, -0.15, -5], palette.floor);
  for (let step = 0; step < 8; step++) {
    const height = (step + 1) * 0.25;
    box(
      `Stair ${step + 1}`,
      [3.4, height, 0.65],
      [-7, height / 2, 2.8 - step * 0.65],
      palette.steps,
    );
  }
  box("Stair landing", [3.4, 2, 1.5], [-7, 1, -2.8], palette.steps);
  label("STAIRS  /  0.25 m", -7, 4);

  function ramp(
    name: string,
    x: number,
    z: number,
    length: number,
    angle: number,
    color: string,
  ) {
    const radians = THREE.MathUtils.degToRad(angle);
    const height = (Math.sin(radians) * length) / 2 + 0.1;
    const body = box(name, [3, 0.2, length], [x, height, z], color);
    body.rotation.x = radians;
    return height * 2;
  }
  const rampHeight = ramp("25° ramp", -0.5, -0.2, 6, 25, palette.slope);
  box(
    "Ramp landing",
    [3, rampHeight, 1.3],
    [-0.5, rampHeight / 2, -3.55],
    palette.slope,
  );
  ramp("58° ramp", 5.5, 0.7, 4, 58, palette.steep);
  label("RAMP  /  25°", -0.5, 4);
  label("STEEP  /  58°", 5.5, 4);

  box("Tunnel roof", [3.8, 0.3, 4], [-7, 1.3, -9], palette.tunnel);
  box("Tunnel left wall", [0.3, 1.45, 4], [-8.75, 0.725, -9], palette.tunnel);
  box("Tunnel right wall", [0.3, 1.45, 4], [-5.25, 0.725, -9], palette.tunnel);
  label("CROUCH  /  1.15 m", -7, -5.9);

  box("Corner back", [5, 2.6, 0.3], [8, 1.3, -6], palette.walls);
  box("Corner side", [0.3, 2.6, 4], [10.35, 1.3, -4.15], palette.walls);
  label("WALL SLIDE", 8, -1.2);

  const elevator = box(
    "Elevator",
    [3.6, 0.25, 3.6],
    [-0.5, 0.125, -9],
    palette.platform,
    true,
  );
  const turntable = box(
    "Rotating platform",
    [4, 0.25, 4],
    [6.5, 0.125, -13],
    palette.platform,
    true,
  );
  const stripeMaterial = new THREE.MeshStandardMaterial({ color: "#d7e9e7" });
  for (const body of [elevator, turntable]) {
    const stripe = new THREE.Mesh(
      new THREE.BoxGeometry(0.15, 0.005, 3.2),
      stripeMaterial,
    );
    stripe.position.y = 0.128;
    body.add(stripe);
  }
  label("ELEVATOR", -0.5, -6.4);
  label("ROTATING PLATFORM", 6.5, -9.7);
  label("START", 0, 9.2);

  const pose = new THREE.Matrix4();
  const quaternion = new THREE.Quaternion();
  const up = new THREE.Vector3(0, 1, 0);
  const position = new THREE.Vector3();
  const scale = new THREE.Vector3(1, 1, 1);
  function update(time: number) {
    position.set(-0.5, 0.125 + 1.3 * (1 - Math.cos(time * 0.75)), -9);
    pose.compose(position, quaternion.identity(), scale);
    elevator.setKinematicTarget(pose);
    position.set(6.5, 0.125, -13);
    pose.compose(position, quaternion.setFromAxisAngle(up, time * 0.45), scale);
    turntable.setKinematicTarget(pose);
  }

  return {
    root,
    spawn,
    labels,
    platforms: { elevator, turntable },
    update,
    reset: () => update(0),
  };
}
