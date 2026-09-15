import * as THREE from "three";
import {
  RigidBody,
  BoxCollider,
  MeshCollider,
  type PhysicsWorld,
} from "@drawcall/physics";
import { box } from "./model";

export function createRoad(world: PhysicsWorld) {
  const root = new THREE.Group();
  const asphalt = new THREE.MeshStandardMaterial({
    color: "#526478",
    roughness: 1,
  });
  const yellow = new THREE.MeshStandardMaterial({
    color: "#e5b550",
    roughness: 0.8,
  });
  const white = new THREE.MeshStandardMaterial({
    color: "#d2d8d7",
    roughness: 1,
  });
  const floor = new RigidBody({ world, type: "static" });
  floor.name = "Road";
  floor.position.set(0, -0.15, 30);
  const floorCollider = new BoxCollider({ size: [8, 0.3, 90] });
  floorCollider.setCollisionGroups({ membership: 1, filter: 2 });
  floor.add(box([8, 0.3, 90], asphalt), floorCollider);
  root.add(floor);
  for (const x of [-3.6, 3.6]) {
    const line = box([0.09, 0.012, 90], white);
    line.position.set(x, 0.007, 30);
    root.add(line);
  }
  for (let z = -10; z < 75; z += 5) {
    const line = box([0.08, 0.012, 1.8], white);
    line.position.set(2.6, 0.007, z);
    root.add(line);
  }
  function bump(
    name: string,
    z: number,
    height: number,
    length: number,
    x = 0,
    width = 6,
  ) {
    const vertices: number[] = [];
    const indices: number[] = [];
    for (let i = 0; i <= 24; i++) {
      const localZ = (i / 24 - 0.5) * length;
      const y = (height * (1 + Math.cos((localZ / length) * 2 * Math.PI))) / 2;
      vertices.push(-width / 2, y, localZ, width / 2, y, localZ);
      if (i < 24) {
        const a = i * 2;
        indices.push(a, a + 2, a + 1, a + 1, a + 2, a + 3);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(vertices, 3),
    );
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    const body = new RigidBody({ world, type: "static" });
    body.name = name;
    body.position.set(x, 0.005, z);
    const mesh = new THREE.Mesh(geometry, yellow);
    const collider = new MeshCollider({ approximation: "trimesh" });
    collider.setGeometry(geometry);
    collider.setCollisionGroups({ membership: 1, filter: 2 });
    body.add(mesh, collider);
    root.add(body);
  }
  bump("Low speed hump 80mm", 12, 0.08, 1.5);
  bump("Speed hump 140mm", 22, 0.14, 2);
  bump("Left wheel bump 120mm", 31, 0.12, 1.3, -0.95, 1.1);
  bump("Right wheel bump 120mm", 35, 0.12, 1.3, 0.95, 1.1);
  for (let i = 0; i < 6; i++)
    bump(`Washboard ${i + 1}`, 44 + i * 1.1, 0.045, 0.65);
  const finish = box([7.2, 0.012, 0.25], white);
  finish.position.set(0, 0.009, 59);
  root.add(finish);
  return root;
}
