import * as THREE from "three";
import { BoxCollider, RigidBody } from "@drawcall/physics";

export function createField(flying: boolean) {
  const root = new THREE.Group();
  const materials = new Map<string, THREE.MeshStandardMaterial>();
  function box(
    name: string,
    size: [number, number, number],
    position: [number, number, number],
    color: string,
    mass?: number,
  ) {
    let material = materials.get(color);
    if (!material) {
      material = new THREE.MeshStandardMaterial({ color, roughness: 0.85 });
      materials.set(color, material);
    }
    const body = new RigidBody({
      bodyType: mass ? "dynamic" : "static",
      colliders: false,
      ...(mass === undefined ? {} : { mass }),
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
  box("Field", [600, 1, 600], [0, -0.5, -80], "#b5c4ac");
  box("Runway", [30, 0.04, 240], [0, 0.02, -80], "#748981");
  for (let z = 15; z > -190; z -= 12) {
    const stripe = new THREE.Mesh(
      new THREE.PlaneGeometry(0.2, 4),
      new THREE.MeshBasicMaterial({ color: "#e9e6d2" }),
    );
    stripe.rotation.x = -Math.PI / 2;
    stripe.position.set(0, 0.045, z);
    root.add(stripe);
  }
  if (flying) {
    const ringMaterial = new THREE.MeshStandardMaterial({
      color: "#e4a452",
      roughness: 0.5,
    });
    const gates: [number, number, number][] = [
      [0, 8, -35],
      [8, 12, -85],
      [-10, 16, -145],
    ];
    for (const [x, y, z] of gates) {
      const ring = new THREE.Mesh(
        new THREE.TorusGeometry(5, 0.18, 8, 48),
        ringMaterial,
      );
      ring.position.set(x, y, z);
      root.add(ring);
      box("Gate post", [0.3, y, 0.3], [x - 5.6, y / 2, z], "#819888");
      box("Gate post", [0.3, y, 0.3], [x + 5.6, y / 2, z], "#819888");
    }
    box("Landing pad", [12, 0.3, 12], [22, 0.15, -25], "#e4a452");
    return root;
  }
  for (let i = 0; i < 6; i++) {
    const x = i % 2 === 0 ? -4 : 4;
    box("Slalom marker", [1, 1.6, 1], [x, 0.8, -18 - i * 12], "#d99a51", 30);
  }
  const ramp = box("Jump ramp", [6, 0.4, 9], [-22, 0.95, -22], "#d19a77");
  ramp.rotation.x = 0.22;
  for (let i = 0; i < 5; i++)
    box("Bump", [6, 0.12, 0.5], [22, 0.06, -14 - i * 2.5], "#d8cba6");
  for (let i = 0; i < 4; i++)
    box(
      "Pushable crate",
      [1.5, 1.5, 1.5],
      [-10 + i * 2, 0.75, -95],
      "#bd815e",
      55,
    );
  return root;
}
