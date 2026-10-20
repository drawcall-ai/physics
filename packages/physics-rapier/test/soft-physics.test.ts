import { expect, it } from "vitest";
import {
  BufferGeometry,
  Float32BufferAttribute,
  PlaneGeometry,
  Vector3,
} from "three";
import { SoftAttachment, SoftBody } from "@drawcall/physics";
import { SoftBinding } from "../src/soft/body.js";
import { createWorld, steps } from "./fixtures.js";

const spring = {
  model: "spring",
  stretch: { frequency: 0, dampingRatio: 0 },
} as const;

it("converges in timestep for a fixed cloth mesh", async () => {
  const sag: number[] = [];
  for (const fixedDelta of [1 / 120, 1 / 240]) {
    const world = await createWorld({
      fixedDelta,
      gravity: [0, -9.81, 0],
      solverIterations: 8,
    });
    const body = new SoftBody({
      shape: {
        kind: "surface",
        geometry: new PlaneGeometry(1, 1, 4, 4).rotateX(-Math.PI / 2),
        thickness: 0.002,
      },
      material: {
        density: 500,
        elasticity: {
          model: "spring",
          stretch: { frequency: 30, dampingRatio: 1 },
          bend: { frequency: 0.5, dampingRatio: 0.5 },
        },
      },
    });
    const region = Array.from({ length: 25 }, (_, i) => i).filter(
      (i) => i < 5 || i >= 20 || i % 5 === 0 || i % 5 === 4,
    );
    world.scene.add(body, new SoftAttachment({ body, region, target: null }));
    for (let i = 0; i < Math.round(4 / fixedDelta); i++) {
      // Keep the native island awake so sleep thresholds cannot substitute for equilibrium.
      body.applyForce(new Vector3(), new Vector3());
      world.update(fixedDelta);
    }
    const positions = body.getPositions();
    expect(positions.every(Number.isFinite)).toBe(true);
    sag.push(-Math.min(...positions.filter((_, i) => i % 3 === 1)));
  }
  const coarse = sag[0],
    fine = sag[1];
  if (coarse === undefined || fine === undefined)
    throw new Error("Missing cloth result");
  expect(fine).toBeGreaterThan(0);
  expect(Math.abs(coarse / fine - 1)).toBeLessThan(0.005);
});

it("integrates density over unequal elements instead of assigning equal node masses", async () => {
  const api = await import("@dimforge/rapier3d-compat");
  await api.init();
  const simulation = new api.World({ x: 0, y: 0, z: 0 });
  const surface = new BufferGeometry();
  surface.setAttribute(
    "position",
    new Float32BufferAttribute([0, 0, 0, 2, 0, 0, 0, 1, 0, 0, -3, 0], 3),
  );
  surface.setIndex([0, 1, 2, 1, 0, 3]);
  const cases = [
    {
      body: new SoftBody({
        shape: {
          kind: "curve",
          points: [
            [0, 0, 0],
            [0.1, 0, 0],
            [1, 0, 0],
          ],
          radius: 0.1,
        },
        material: { density: 1 / (Math.PI * 0.1 ** 2), elasticity: spring },
      }),
      masses: [0.05, 0.5, 0.45],
      center: [0.5, 0, 0],
    },
    {
      body: new SoftBody({
        shape: { kind: "surface", geometry: surface, thickness: 0.1 },
        material: { density: 10, elasticity: spring },
      }),
      masses: [4 / 3, 4 / 3, 1 / 3, 1],
      center: [2 / 3, -2 / 3, 0],
    },
    {
      body: new SoftBody({
        shape: {
          kind: "volume",
          positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -3],
          tetrahedra: [0, 1, 2, 3, 0, 2, 1, 4],
        },
        material: {
          density: 6,
          elasticity: {
            model: "corotational",
            youngModulus: 6000,
            poissonRatio: 0,
          },
        },
      }),
      masses: [1, 1, 1, 0.25, 0.75],
      center: [0.25, 0.25, -0.5],
    },
  ];
  try {
    for (const { body, masses, center } of cases) {
      const { native } = new SoftBinding(api, simulation, body);
      masses.forEach((mass, node) =>
        expect(native.particleMass(node)).toBeCloseTo(mass, 5),
      );
      expect(native.mass()).toBeCloseTo(
        masses.reduce((sum, mass) => sum + mass, 0),
        5,
      );
      const actual = new Vector3().copy(native.centerOfMass()).toArray();
      center.forEach((value, axis) =>
        expect(actual[axis]).toBeCloseTo(value, 5),
      );
    }
  } finally {
    simulation.free();
  }
});

it.each([
  { youngModulus: 6000, poissonRatio: 0, density: 100 },
  { youngModulus: 12000, poissonRatio: 0, density: 100 },
  { youngModulus: 6000, poissonRatio: 0.3, density: 1000 },
])(
  "matches a tetrahedron's analytical static compliance: $youngModulus Pa, ν=$poissonRatio",
  async ({ youngModulus, poissonRatio, density }) => {
    const world = await createWorld({ fixedDelta: 0.001, solverIterations: 8 });
    const body = new SoftBody({
      shape: {
        kind: "volume",
        positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        tetrahedra: [0, 1, 2, 3],
      },
      material: {
        density,
        elasticity: {
          model: "corotational",
          youngModulus,
          poissonRatio,
          dampingRatio: 0.7,
        },
      },
    });
    world.scene.add(
      body,
      new SoftAttachment({ body, region: [0, 1, 2], target: null }),
    );
    for (let i = 0; i < 3000; i++) {
      body.applyForce(new Vector3(0, 0, 10), new Vector3(0, 0, 1));
      world.update(world.fixedDelta);
    }
    // Fixed base, ν-dependent constrained longitudinal modulus, V=1/6 m³, L=1 m.
    const expected =
      (60 * (1 + poissonRatio) * (1 - 2 * poissonRatio)) /
      (youngModulus * (1 - poissonRatio));
    const tip = new Vector3().fromArray(body.getPositions(), 9);
    expect(Math.abs((tip.z - 1) / expected - 1)).toBeLessThan(0.01);
  },
);

it.each([0.01, 0.002])(
  "damps whole-body translation at a mass-independent rate with dt=%s",
  async (fixedDelta) => {
    const world = await createWorld({ fixedDelta });
    for (const mass of [1, 10]) {
      const body = new SoftBody({
        shape: {
          kind: "curve",
          points: [
            [0, mass, 0],
            [1, mass, 0],
          ],
          radius: 0.1,
        },
        material: { density: mass / (Math.PI * 0.1 ** 2), elasticity: spring },
      });
      world.scene.add(body);
      body.applyImpulse(new Vector3(mass, 0, 0));
      steps(world, Math.round(1 / fixedDelta));
      const before = body.getPositions();
      body.setLinearDamping(1);
      world.update(0);
      expect(body.getPositions()).toEqual(before);
      steps(world, Math.round(1 / fixedDelta));
      const atOneSecond = new Vector3().fromArray(body.getPositions());
      steps(world, 1);
      const next = new Vector3().fromArray(body.getPositions());
      // Native backward-Euler damping converges to v(t)=v(0) exp(-rate*t).
      expect((next.x - atOneSecond.x) / fixedDelta).toBeCloseTo(
        Math.exp(-1),
        2,
      );
      expect(atOneSecond.x - new Vector3().fromArray(before).x).toBeCloseTo(
        1 - Math.exp(-1),
        2,
      );
      body.removeFromParent();
    }
  },
);

it("rejects Poisson ratios the native material would silently clamp", async () => {
  const world = await createWorld();
  const body = new SoftBody({
    shape: {
      kind: "volume",
      positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
      tetrahedra: [0, 1, 2, 3],
    },
    material: {
      density: 100,
      elasticity: {
        model: "corotational",
        youngModulus: 6000,
        poissonRatio: 0.4999,
      },
    },
  });
  world.scene.add(body);
  expect(() => world.update(0)).toThrow("Poisson ratios up to 0.499");
});
