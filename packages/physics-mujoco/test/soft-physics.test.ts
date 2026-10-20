import { expect, test } from "vitest";
import { SoftBody, SoftAttachment } from "@drawcall/physics";
import { PlaneGeometry, Vector3 } from "three";
import { scene, createWorld, steps } from "./fixtures.js";

function tetrahedron(
  density: number,
  youngModulus: number,
  dampingTime: number,
) {
  return new SoftBody({
    shape: {
      kind: "volume",
      positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
      tetrahedra: [0, 1, 2, 3],
    },
    material: {
      density,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus,
        poissonRatio: 0,
        dampingTime,
      },
    },
  });
}
function x(body: SoftBody): number {
  const value = body.getPositions()[0];
  if (value === undefined) throw new Error("Missing first node");
  return value;
}

test.each([0.01, 0.001])(
  "linear damping at %s s steps follows the authored decay independently of shape and mass",
  async (dt) => {
    const light = tetrahedron(6, 10000, 1);
    const cases = [
      { body: light, mass: 1 },
      { body: tetrahedron(60, 10000, 1), mass: 10 },
      {
        body: new SoftBody({
          shape: {
            kind: "surface",
            geometry: new PlaneGeometry(1, 1),
            thickness: 0.01,
          },
          material: { ...light.material, density: 100 },
        }),
        mass: 1,
      },
      {
        body: new SoftBody({
          shape: {
            kind: "curve",
            points: [
              [0, 0, 0],
              [1, 0, 0],
            ],
            radius: 0.1,
          },
          material: {
            density: 100,
            elasticity: {
              model: "spring",
              stretch: { frequency: 10, dampingRatio: 1 },
            },
          },
        }),
        mass: Math.PI,
      },
    ];
    for (const [index, { body }] of cases.entries()) {
      body.position.z = index * 2;
      scene.add(body);
    }
    const world = await createWorld({ fixedDelta: dt, solverIterations: 100 });
    const starts = cases.map(({ body }) => x(body));
    // Analytic shape volumes give these masses; every impulse should produce 1 m/s.
    for (const { body, mass } of cases)
      body.applyImpulse(new Vector3(mass, 0, 0));
    steps(world, Math.round(0.05 / dt));
    for (const [index, { body }] of cases.entries()) {
      const start = starts[index];
      if (start === undefined) throw new Error("Missing initial position");
      // Internal material damping leaves uniform translation unchanged.
      expect(x(body) - start).toBeCloseTo(0.05, 6);
      const before = body.getPositions();
      body.setLinearDamping(2);
      world.update(0);
      expect(Array.from(body.getPositions())).toEqual(Array.from(before));
    }
    const count = Math.round(0.2 / dt);
    steps(world, count - 1);
    const previous = cases.map(({ body }) => x(body));
    steps(world, 1);
    const rate = 2;
    const decay = (1 + rate * dt) ** -count;
    const displacement = (1 - decay) / rate;
    // The free-body discrete decay converges to exp(-rate*time) as the timestep shrinks.
    expect(Math.abs(decay - Math.exp(-rate * 0.2))).toBeLessThan(0.3 * dt);
    for (const [index, { body }] of cases.entries()) {
      const start = starts[index],
        before = previous[index];
      if (start === undefined || before === undefined)
        throw new Error("Missing previous position");
      expect(x(body) - start).toBeCloseTo(0.05 + displacement, 6);
      expect((x(body) - before) / dt).toBeCloseTo(decay, 3);
    }
  },
);

test("Young's modulus in pascals gives the analytical single-tetrahedron SVK tensile equilibrium", async () => {
  const body = tetrahedron(6, 600, 0.1);
  scene.add(
    body,
    new SoftAttachment({ body, region: [0, 2, 3], target: null }),
  );
  const world = await createWorld({ fixedDelta: 0.001, solverIterations: 100 });
  for (let i = 0; i < 2000; i++) {
    body.applyForce(new Vector3(1, 0, 0), new Vector3(1, 0, 0));
    steps(world, 1);
  }
  // ν=0, L=1, V=1/6: F = E*V/(2L) * (stretch³-stretch).
  let stretch = 1;
  for (let i = 0; i < 8; i++)
    stretch -=
      (50 * (stretch ** 3 - stretch) - 1) / (50 * (3 * stretch ** 2 - 1));
  const tip = body.getPositions()[3];
  if (tip === undefined) throw new Error("Missing loaded node");
  expect(tip - 1).toBeCloseTo(stretch - 1, 4);
});

test.each([
  [1, 0.01],
  [1, 0.002],
  [4, 0.002],
])(
  "a membrane tensile patch matches SVK with %s subdivisions and %s s steps",
  async (divisions, dt) => {
    const geometry = new PlaneGeometry(1, 1, divisions, divisions);
    geometry.translate(0.5, 0, 0);
    const body = new SoftBody({
      shape: {
        kind: "surface",
        geometry,
        thickness: 0.01,
      },
      material: {
        density: 100,
        elasticity: {
          model: "saintVenantKirchhoff",
          youngModulus: 10000,
          poissonRatio: 0,
          dampingTime: 0.1,
        },
      },
    });
    scene.add(
      body,
      new SoftAttachment({
        body,
        region: { min: [0, -1, -1], max: [0, 1, 1] },
        target: null,
      }),
    );
    const world = await createWorld({ fixedDelta: dt, solverIterations: 100 });
    const rest = body.getPositions();
    // ν=0, E=10 kPa, cross-section=.01 m²: F=E*A/2*(stretch³-stretch).
    const stretch = 1.01;
    const force = 50 * (stretch ** 3 - stretch);
    for (let step = 0; step < Math.round(4 / dt); step++) {
      for (let row = 0; row <= divisions; row++) {
        const weight = row === 0 || row === divisions ? 0.5 : 1;
        body.applyForce(
          new Vector3((force * weight) / divisions, 0, 0),
          new Vector3(1, 0.5 - row / divisions, 0),
        );
      }
      steps(world, 1);
    }
    // This affine solution is exact on either mesh.
    const positions = body.getPositions();
    for (let node = 0; node < rest.length; node += 3) {
      const expected = new Vector3().fromArray(rest, node);
      expected.x *= stretch;
      const actual = new Vector3().fromArray(positions, node);
      expect(actual.distanceTo(expected) / (stretch - 1)).toBeLessThan(0.001);
    }
  },
);
