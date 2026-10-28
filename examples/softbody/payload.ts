import {
  BoxGeometry,
  DoubleSide,
  Group,
  Mesh,
  MeshStandardMaterial,
  PlaneGeometry,
} from "three";
import { RigidBody, SoftBody, type Vec3 } from "@drawcall/physics";

export type PayloadKind = "block" | "cloth" | "rope";

export function payload(kind: PayloadKind) {
  const root = new Group();
  let body: RigidBody | SoftBody;
  if (kind === "block") {
    body = new RigidBody({
      mass: 0.05,
      material: { staticFriction: 0.7, dynamicFriction: 0.7 },
    });
    body.position.y = 0.03;
    body.add(
      new Mesh(
        new BoxGeometry(0.06, 0.06, 0.06),
        new MeshStandardMaterial({ color: "#efae67", roughness: 0.8 }),
      ),
    );
  } else if (kind === "cloth") {
    const geometry = new PlaneGeometry(0.06, 0.14, 3, 6).rotateX(-Math.PI / 2);
    body = new SoftBody({
      shape: { kind: "surface", geometry, thickness: 0.001 },
      material: {
        density: 300,
        dynamicFriction: 0.8,
        elasticity: {
          model: "saintVenantKirchhoff",
          youngModulus: 200000,
          poissonRatio: 0.3,
          dampingTime: 0.002,
        },
      },
      selfCollision: true,
    });
    geometry.dispose();
    body.position.y = 0.042;
    body.setLinearDamping(0.5);
    body.add(
      new Mesh(
        body.geometry,
        new MeshStandardMaterial({
          color: "#ee8e8b",
          roughness: 0.9,
          side: DoubleSide,
        }),
      ),
    );
    root.add(support([0.04, 0.04, 0.05]));
  } else {
    const points: Vec3[] = Array.from({ length: 21 }, (_, i) => [
      0,
      0,
      (i / 20 - 0.5) * 0.24,
    ]);
    body = new SoftBody({
      shape: { kind: "curve", points, radius: 0.004 },
      material: {
        density: 900,
        dynamicFriction: 0.7,
        elasticity: {
          model: "spring",
          stretch: { frequency: 40, dampingRatio: 1 },
        },
      },
      selfCollision: true,
    });
    body.position.y = 0.036;
    body.setLinearDamping(0.3);
    body.add(
      new Mesh(
        body.geometry,
        new MeshStandardMaterial({ color: "#9f8ee8", roughness: 0.8 }),
      ),
    );
    root.add(support([0.002, 0.032, 0.05]));
  }
  body.name = kind;
  root.add(body);
  return {
    root,
    body,
    travel: kind === "rope" ? 0.058 : 0.032,
    dispose() {
      root.traverse((object) => {
        if (!(object instanceof Mesh)) return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        for (const material of materials) material.dispose();
      });
    },
  };
}

function support(size: Vec3) {
  const body = new RigidBody({ bodyType: "static" });
  body.position.y = size[1] / 2;
  body.add(
    new Mesh(
      new BoxGeometry(...size),
      new MeshStandardMaterial({ color: "#69808b", roughness: 0.8 }),
    ),
  );
  return body;
}
