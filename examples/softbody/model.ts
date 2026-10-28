import {
  BoxGeometry,
  GridHelper,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Scene,
} from "three";
import { RigidBody, SoftAttachment, SoftBody } from "@drawcall/physics";
import { FEMLoader } from "@drawcall/physics-fem";
import type { MujocoWorldOptions } from "@drawcall/physics-mujoco";
import { thickness, height, width } from "./public/pad.ts";
import { payload, type PayloadKind } from "./payload.ts";

export const simulationOptions = {
  fixedDelta: 1 / 200,
  maxSubsteps: 20,
  solverIterations: 300,
  frictionCone: "elliptic",
  frictionImpedanceRatio: 50,
} satisfies Omit<MujocoWorldOptions, "scene">;

export async function gripper(url: string, kind: PayloadKind = "block") {
  const scene = new Scene();
  const loader = new FEMLoader();
  const geometry = new BoxGeometry(1, 1, 1);
  const metal = new MeshStandardMaterial({
    color: "#8a9eac",
    roughness: 0.35,
    metalness: 0.6,
  });
  const surface = new MeshStandardMaterial({
    color: "#233640",
    roughness: 0.9,
  });
  const floor = new RigidBody({ bodyType: "static" });
  floor.position.y = -0.005;
  const floorMesh = new Mesh(geometry, surface);
  floorMesh.scale.set(0.5, 0.01, 0.35);
  floor.add(floorMesh);
  const item = payload(kind);
  const grid = new GridHelper(0.3, 15, "#59707b", "#3b515d");
  grid.position.y = 0.0002;
  const housing = new Mesh(geometry, metal);
  housing.scale.set(0.2, 0.025, 0.075);
  housing.position.y = 0.13;
  scene.add(floor, item.root, housing, grid);
  const jaws: RigidBody[] = [];
  const pads: SoftBody[] = [];
  for (const side of [-1, 1]) {
    const pad = await loader.loadAsync({
      url,
      material: {
        density: 1100,
        dynamicFriction: 0.7,
        elasticity: {
          model: "linear",
          youngModulus: 25000,
          poissonRatio: 0.3,
          dampingTime: 0.03,
        },
      },
      mount: { min: [-0.0001, -height, -width], max: [0.0001, height, width] },
      contact: { min: [0, -height, -width], max: [thickness, height, width] },
      mesh: { cellSize: 0.02, order: 2, maxDofs: 90 },
    });
    if (!pad.model) throw new Error("Missing reduced pad model");
    pad.name = side < 0 ? "Left pad" : "Right pad";
    pad.position.set(side * (0.06 + thickness), 0.055, 0);
    if (side > 0) pad.rotation.y = Math.PI;
    const jaw = new RigidBody({ bodyType: "kinematic", mass: 1 });
    jaw.position.copy(pad.position);
    const backing = new Mesh(geometry, metal);
    backing.scale.set(0.012, 0.12, width + 0.01);
    backing.position.set(side * 0.006, 0.02, 0);
    jaw.add(backing);
    scene.add(
      pad,
      jaw,
      new SoftAttachment({ body: pad, region: pad.model.mount, target: jaw }),
    );
    jaws.push(jaw);
    pads.push(pad);
  }
  let grip = 0,
    lift = 0;
  const target = new Matrix4();
  scene.traverse((object) => {
    if (!(object instanceof Mesh)) return;
    object.castShadow = object !== floorMesh;
    object.receiveShadow = true;
  });
  return {
    scene,
    pads,
    object: item.body,
    height() {
      if (!(item.body instanceof SoftBody)) return item.body.position.y;
      const positions = item.body.getPositions();
      return (
        positions.reduce(
          (sum, value, i) => sum + (i % 3 === 1 ? value : 0),
          0,
        ) /
        (positions.length / 3)
      );
    },
    update(input: { grip: number; lift: number }, dt: number) {
      grip += Math.max(-dt, Math.min(dt, input.grip - grip));
      lift += Math.max(-dt / 2, Math.min(dt / 2, input.lift - lift));
      housing.position.y = 0.13 + lift * 0.16;
      for (const [i, jaw] of jaws.entries()) {
        const side = i === 0 ? -1 : 1;
        jaw.setKinematicTarget(
          target.makeTranslation(
            side * (0.06 + thickness - grip * item.travel),
            0.055 + lift * 0.16,
            0,
          ),
        );
      }
    },
    reset() {
      grip = 0;
      lift = 0;
      housing.position.y = 0.13;
    },
    dispose() {
      for (const pad of pads) {
        pad.geometry.dispose();
        pad.traverse((object) => {
          if (object instanceof Mesh) object.geometry.dispose();
        });
      }
      loader.dispose();
      item.dispose();
      geometry.dispose();
      metal.dispose();
      surface.dispose();
      grid.geometry.dispose();
      grid.material.dispose();
    },
  };
}
