import {
  BoxGeometry,
  CylinderGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  Vector3,
} from "three";
import { BoxCollider, RigidBody } from "@drawcall/physics";

export const tuning = {
  mass: 900,
  radius: 0.32,
  suspension: 0.55,
  spring: 22000,
  damping: 2800,
  grip: 1.7,
  driftGrip: 0.35,
  cornering: 2200,
  driveForce: 1800,
  brakeForce: 3000,
  forwardSpeed: 24,
  reverseSpeed: 10,
  steerAngle: 0.55,
};

export function createCar() {
  const body = new RigidBody({
    mass: tuning.mass,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1100, 1400, 650],
    colliders: false,
    canSleep: false,
  });
  body.name = "Raycast car";
  body.position.y = 0.85;
  const collider = new BoxCollider({ size: [1.5, 0.4, 3.3] });
  collider.position.y = 0.15;
  body.add(collider);

  const paint = new MeshStandardMaterial({ color: "#dc7744", roughness: 0.4 });
  const glass = new MeshStandardMaterial({ color: "#30494f", roughness: 0.3 });
  const rubber = new MeshStandardMaterial({ color: "#293332", roughness: 0.9 });
  const silver = new MeshStandardMaterial({ color: "#dbe2d7", roughness: 0.5 });
  function box(size: [number, number, number], material: MeshStandardMaterial) {
    const mesh = new Mesh(new BoxGeometry(...size), material);
    mesh.castShadow = mesh.receiveShadow = true;
    body.add(mesh);
    return mesh;
  }
  box([1.5, 0.4, 3.3], paint).position.y = 0.15;
  box([1.25, 0.48, 1.5], glass).position.set(0, 0.58, 0.25);
  for (const x of [-0.52, 0.52])
    box([0.3, 0.1, 0.03], silver).position.set(x, 0.19, -1.66);

  const wheels = [];
  for (const z of [-1.2, 1.2]) {
    for (const x of [-0.86, 0.86]) {
      const mount = new Vector3(x, 0, z);
      const pivot = new Group();
      const spin = new Group();
      pivot.position.copy(mount).add(new Vector3(0, -tuning.suspension, 0));
      const mesh = new Mesh(
        new CylinderGeometry(tuning.radius, tuning.radius, 0.24, 20),
        rubber,
      );
      mesh.rotation.z = Math.PI / 2;
      mesh.castShadow = mesh.receiveShadow = true;
      const hub = new Mesh(new BoxGeometry(0.255, 0.06, 0.44), silver);
      spin.add(mesh, hub);
      pivot.add(spin);
      body.add(pivot);
      wheels.push({ mount, pivot, spin, front: z < 0 });
    }
  }
  return { body, wheels };
}

export type Car = ReturnType<typeof createCar>;
