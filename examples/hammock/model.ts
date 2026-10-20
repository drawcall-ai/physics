import * as THREE from "three";
import {
  RigidBody,
  SoftAttachment,
  SoftBody,
  type SoftMaterial,
  type Vec3,
} from "@drawcall/physics";

import { volume } from "./volume";

export type Backend = "rapier" | "mujoco";
export const dampingRate = 0.2;

export function setDamping(root: THREE.Object3D, rate: number): void {
  if (!Number.isFinite(rate) || rate < 0)
    throw new Error("Motion damping must be finite and nonnegative");
  root.traverse((object) => {
    if (object instanceof SoftBody) object.setLinearDamping(rate);
    if (object instanceof RigidBody && object.bodyType === "dynamic")
      object.setLinearDamping(rate).setAngularDamping(rate);
  });
}

const rapierOptions = {
  fixedDelta: 1 / 120,
  maxSubsteps: 8,
  solverIterations: 8,
};

const fabricProperties = {
  density: 500,
  dynamicFriction: 0.6,
  staticFriction: 0.6,
};
const rapierFabrics: Record<string, SoftMaterial> = {
  canvas: {
    ...fabricProperties,
    elasticity: {
      model: "spring",
      stretch: { frequency: 30, dampingRatio: 1 },
      bend: { frequency: 0.5, dampingRatio: 0.5 },
    },
  },
  stretch: {
    ...fabricProperties,
    elasticity: {
      model: "spring",
      stretch: { frequency: 12, dampingRatio: 1 },
      bend: { frequency: 0.2, dampingRatio: 0.5 },
    },
  },
};
export function simulationOptions(backend: Backend) {
  return backend === "rapier"
    ? rapierOptions
    : {
        fixedDelta: 1 / 1000,
        maxSubsteps: 20,
        solverIterations: 100,
      };
}

// These elastic laws are illustrative profiles, not calibrated equivalents.
const mujocoFabrics: Record<string, SoftMaterial> = {
  canvas: {
    ...fabricProperties,
    elasticity: {
      model: "saintVenantKirchhoff",
      youngModulus: 1500000,
      poissonRatio: 0.3,
      dampingTime: 0.001,
    },
  },
  stretch: {
    ...fabricProperties,
    elasticity: {
      model: "saintVenantKirchhoff",
      youngModulus: 800000,
      poissonRatio: 0.3,
      dampingTime: 0.001,
    },
  },
};

export function fabrics(backend: Backend): Record<string, SoftMaterial> {
  return backend === "rapier" ? rapierFabrics : mujocoFabrics;
}

const ropeMaterial: SoftMaterial = {
  density: 800,
  elasticity: {
    model: "spring",
    stretch: { frequency: 90, dampingRatio: 1 },
  },
};
const gold = new THREE.MeshStandardMaterial({
  color: "#efbd68",
  roughness: 0.4,
});
const steel = new THREE.MeshStandardMaterial({
  color: "#536d80",
  metalness: 0.5,
  roughness: 0.5,
});

function box(size: Vec3, material: THREE.Material) {
  return new THREE.Mesh(new THREE.BoxGeometry(...size), material);
}

export function hammock(backend: Backend) {
  const root = new THREE.Group();
  const frame = new RigidBody({ bodyType: "static" });
  frame.name = "Frame";
  for (const x of [-2.1, 2.1]) {
    for (const z of [-1.2, 1.2]) {
      const post = box([0.1, 3.8, 0.1], steel);
      post.position.set(x, 1.9, z);
      frame.add(post);
    }
    const beam = box([0.12, 0.12, 2.5], steel);
    beam.position.set(x, 3.7, 0);
    frame.add(beam);
  }
  const floor = new RigidBody({ bodyType: "static" });
  floor.name = "Floor";
  floor.position.y = -0.15;
  floor.add(
    box(
      [14, 0.2, 14],
      new THREE.MeshStandardMaterial({ color: "#24333e", roughness: 1 }),
    ),
  );
  root.add(frame, floor);

  const fabric = fabrics(backend).canvas;
  if (!fabric) throw new Error("Missing canvas material");
  const geometry = new THREE.PlaneGeometry(3.2, 2, 12, 8);
  geometry.rotateX(-Math.PI / 2);
  const cloth = new SoftBody({
    shape: {
      kind: "surface",
      geometry,
      thickness: 0.002,
    },
    material: fabric,
    selfCollision: true,
  });
  cloth.name = "Hammock";
  cloth.position.y = 1.9;
  const fabricVisual = new THREE.MeshStandardMaterial({
    color: "#64c7b3",
    side: THREE.DoubleSide,
    roughness: 0.9,
  });
  cloth.add(new THREE.Mesh(cloth.geometry, fabricVisual));
  root.add(cloth);

  const bars: RigidBody[] = [];
  const ropes: SoftBody[] = [];
  const anchors: {
    marker: THREE.Mesh;
    parent: RigidBody;
    point: THREE.Vector3;
  }[] = [];
  for (const side of [-1, 1]) {
    const bar = new RigidBody({ mass: 0.5 });
    bar.name = side < 0 ? "Left bar" : "Right bar";
    const barHeight = 1.84;
    bar.position.set(side * 1.6, barHeight, 0);
    bar.add(box([0.1, 0.1, 2.2], gold));
    bars.push(bar);
    root.add(bar);
    root.add(
      new SoftAttachment({
        body: cloth,
        region: {
          min: [side * 1.6 - 0.01, -0.01, -1.01],
          max: [side * 1.6 + 0.01, 0.01, 1.01],
        },
        target: bar,
      }),
    );
    for (const z of [-1, 1]) {
      const segments = 8;
      const points: Vec3[] = Array.from({ length: segments + 1 }, (_, i) => [
        (side * 0.5 * i) / segments,
        (1.68 * i) / segments,
        0,
      ]);
      const rope = new SoftBody({
        shape: { kind: "curve", points, radius: 0.016 },
        material: ropeMaterial,
      });
      rope.name = "Suspension rope";
      rope.position.set(side * 1.6, 1.92, z);
      rope.add(
        new THREE.Mesh(
          rope.geometry,
          new THREE.MeshStandardMaterial({ color: "#e9ddbd", roughness: 1 }),
        ),
      );
      ropes.push(rope);
      root.add(
        rope,
        new SoftAttachment({ body: rope, region: [0], target: bar }),
        new SoftAttachment({ body: rope, region: [segments], target: frame }),
      );
      for (const [parent, position] of [
        [bar, [0, 1.92 - barHeight, z]],
        [frame, [side * 2.1, 3.6, z]],
      ] satisfies [RigidBody, Vec3][]) {
        const marker = new THREE.Mesh(
          new THREE.SphereGeometry(0.045, 8, 6),
          new THREE.MeshBasicMaterial({ color: "#ffef9e" }),
        );
        marker.visible = false;
        root.add(marker);
        anchors.push({ marker, parent, point: new THREE.Vector3(...position) });
      }
    }
  }
  setDamping(root, dampingRate);
  return { root, cloth, bars, ropes, anchors, fabricVisual };
}

export type Cargo = "rubber" | "foam" | "rigid";

export function cargo(backend: Backend, kind: Cargo, index: number) {
  const position = new THREE.Vector3(
    Math.sin(index * 2.4) * 0.6,
    3,
    Math.cos(index * 2.4) * 0.35,
  );
  const color =
    kind === "rubber" ? "#db816e" : kind === "foam" ? "#a99cdb" : "#d8dfdd";
  const visual = new THREE.MeshStandardMaterial({ color, roughness: 0.65 });
  if (kind === "rigid") {
    const body = new RigidBody({ mass: 0.3 });
    body.name = "Rigid cargo";
    body.add(box([0.35, 0.35, 0.35], visual));
    body.position.copy(position);
    setDamping(body, dampingRate);
    return body;
  }
  const geometry =
    kind === "rubber"
      ? new THREE.IcosahedronGeometry(0.23, 1)
      : new THREE.BoxGeometry(0.45, 0.4, 0.4, 3, 3, 3);
  const elasticity = {
    youngModulus: kind === "rubber" ? 120000 : 20000,
    poissonRatio: 0.3,
  };
  const body = new SoftBody({
    shape: volume(geometry),
    material: {
      density: kind === "rubber" ? 180 : 80,
      staticFriction: 0.6,
      dynamicFriction: 0.6,
      elasticity:
        backend === "rapier"
          ? {
              model: kind === "rubber" ? "neoHookean" : "corotational",
              ...elasticity,
              dampingRatio: 0.1,
            }
          : {
              model: "neoHookean",
              ...elasticity,
              dampingTime: 0,
            },
    },
  });
  geometry.dispose();
  body.name = kind === "rubber" ? "Rubber cargo" : "Foam cargo";
  body.position.copy(position);
  body.add(new THREE.Mesh(body.geometry, visual));
  setDamping(body, dampingRate);
  return body;
}
