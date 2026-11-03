import {
  BoxGeometry,
  CylinderGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
} from "three";
import { BoxCollider, RigidBody, type Vec3 } from "@drawcall/physics";

export const specification = {
  mass: 1,
  inertia: [0.15, 0.25, 0.15] satisfies Vec3,
};

export function createQuadcopter() {
  const body = new RigidBody({
    mass: specification.mass,
    centerOfMass: [0, 0, 0],
    diagonalInertia: specification.inertia,
    colliders: false,
    canSleep: false,
  });
  body.name = "Quadcopter";
  body.position.y = 3;
  body.setMaterial({
    dynamicFriction: 0.4,
    staticFriction: 0.4,
    restitution: 0.1,
  });
  const paint = new MeshStandardMaterial({ color: "#6bafa7", roughness: 0.5 });
  const dark = new MeshStandardMaterial({ color: "#334b51", roughness: 0.5 });
  const center = new Mesh(new BoxGeometry(0.5, 0.22, 0.65), paint);
  const eye = new Mesh(new BoxGeometry(0.22, 0.1, 0.08), dark);
  eye.position.set(0, 0.02, -0.36);
  body.add(center, eye, new BoxCollider({ size: [1.65, 0.22, 1.65] }));
  const rotors: Group[] = [];
  for (const x of [-0.6, 0.6]) {
    for (const z of [-0.6, 0.6]) {
      const arm = new Mesh(new BoxGeometry(0.09, 0.1, Math.hypot(x, z)), dark);
      arm.position.set(x / 2, 0, z / 2);
      arm.rotation.y = Math.atan2(x, z);
      const hub = new Mesh(new CylinderGeometry(0.09, 0.09, 0.15, 12), paint);
      hub.position.set(x, 0.08, z);
      const rotor = new Group();
      rotor.position.set(x, 0.18, z);
      rotor.add(new Mesh(new BoxGeometry(0.6, 0.025, 0.08), dark));
      body.add(arm, hub, rotor);
      rotors.push(rotor);
    }
  }
  body.traverse((object) => {
    if (object instanceof Mesh) object.castShadow = object.receiveShadow = true;
  });
  return { body, rotors };
}

export type Quadcopter = ReturnType<typeof createQuadcopter>;
