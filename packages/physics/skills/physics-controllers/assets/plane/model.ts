import { BoxGeometry, Mesh, MeshStandardMaterial } from "three";
import { BoxCollider, RigidBody, type Vec3 } from "@drawcall/physics";

export const specification = {
  mass: 4,
  inertia: [3, 5, 7] satisfies Vec3,
};

export function createPlane() {
  const body = new RigidBody({
    mass: specification.mass,
    centerOfMass: [0, 0, 0],
    diagonalInertia: specification.inertia,
    velocity: { linear: [0, 0, -18] },
    colliders: false,
    canSleep: false,
  });
  body.name = "Plane";
  body.position.y = 8;
  body.setMaterial({
    dynamicFriction: 0.3,
    staticFriction: 0.3,
    restitution: 0.1,
  });
  const paint = new MeshStandardMaterial({ color: "#e9b950", roughness: 0.5 });
  const dark = new MeshStandardMaterial({ color: "#324d57", roughness: 0.4 });
  function box(size: Vec3, position: Vec3, material = paint) {
    const mesh = new Mesh(new BoxGeometry(...size), material);
    mesh.position.set(...position);
    mesh.castShadow = mesh.receiveShadow = true;
    body.add(mesh);
    return mesh;
  }
  box([0.55, 0.5, 3], [0, 0, 0]);
  box([4.4, 0.12, 0.9], [0, 0.05, -0.35]);
  box([1.7, 0.1, 0.5], [0, 0.15, 1.2]);
  box([0.1, 0.75, 0.55], [0, 0.45, 1.15]);
  box([0.42, 0.3, 0.75], [0, 0.35, -0.35], dark);
  const propeller = box([1.35, 0.09, 0.09], [0, 0, -1.6], dark);
  body.add(new BoxCollider({ size: [0.55, 0.5, 3] }));
  const wing = new BoxCollider({ size: [4.4, 0.12, 0.9] });
  wing.position.set(0, 0.05, -0.35);
  body.add(wing);
  return { body, propeller };
}

export type Plane = ReturnType<typeof createPlane>;
