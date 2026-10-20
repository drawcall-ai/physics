import { expect, test } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { SoftBody } from "@drawcall/physics";
import { PlaneGeometry } from "three";
import { array, at } from "../src/heap.js";
import { compile } from "../src/model/compile.js";
import { softMesh } from "../src/soft/mesh.js";
import { nodeName } from "../src/soft/model.js";
import { createWorld, scene } from "./fixtures.js";

test("SVK membrane forces preserve the authored nonzero Poisson ratio", async () => {
  const api = await loadMujoco();
  const geometry = new PlaneGeometry(1, 1).translate(0.5, 0.5, 0);
  const body = new SoftBody({
    shape: { kind: "surface", geometry, thickness: 0.01 },
    material: {
      density: 100,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus: 10000,
        poissonRatio: 0.3,
      },
    },
  });
  const mesh = softMesh(body);
  const simulation = compile(
    api,
    [],
    new Map(),
    [],
    {
      fixedDelta: 0.001,
      gravity: [0, 0, 0],
      solverIterations: 100,
      frictionCone: "pyramidal",
      frictionImpedanceRatio: 1,
    },
    new Map([[body, mesh]]),
  );
  try {
    const { model, data } = simulation;
    const points = geometry.getAttribute("position");
    const joints = Array.from({ length: points.count }, (_, node) =>
      api.mj_name2id(
        model,
        api.mjtObj.mjOBJ_JOINT.value,
        `${nodeName(mesh, node)}j0`,
      ),
    );
    for (const [node, joint] of joints.entries())
      array(data.qpos).set(
        [0.01 * points.getX(node), 0.02 * points.getY(node), 0],
        at(model.jnt_qposadr, joint),
      );
    api.mj_forward(model, data);
    let forceX = 0,
      forceY = 0;
    for (const [node, joint] of joints.entries()) {
      const dof = at(model.jnt_dofadr, joint);
      if (points.getX(node) === 1) forceX -= at(data.qfrc_passive, dof);
      if (points.getY(node) === 1) forceY -= at(data.qfrc_passive, dof + 1);
    }
    // F = diag(1.01, 1.02, 1), with zero through-thickness strain.
    const mu = 10000 / 2.6,
      lambda = 3000 / (1.3 * 0.4);
    const ex = (1.01 ** 2 - 1) / 2,
      ey = (1.02 ** 2 - 1) / 2;
    expect(forceX).toBeCloseTo(
      0.01 * 1.01 * (2 * mu * ex + lambda * (ex + ey)),
      8,
    );
    expect(forceY).toBeCloseTo(
      0.01 * 1.02 * (2 * mu * ey + lambda * (ex + ey)),
      8,
    );
  } finally {
    simulation.free();
  }
});

test.each(["saintVenantKirchhoff", "neoHookean"] as const)(
  "%s matches analytical tetrahedron forces at millimeter scale and nonzero Poisson ratio",
  async (model) => {
    const api = await loadMujoco();
    const youngModulus = 9.8e6;
    for (const size of [0.001, 0.01])
      for (const poissonRatio of [0, 0.3, 0.45]) {
        const body = new SoftBody({
          shape: {
            kind: "volume",
            positions: [0, 0, 0, size, 0, 0, 0, size, 0, 0, 0, size],
            tetrahedra: [0, 1, 2, 3],
          },
          material: {
            density: 1220,
            elasticity: {
              model,
              youngModulus,
              poissonRatio,
              dampingTime: 0.002,
            },
          },
        });
        const mesh = softMesh(body);
        const simulation = compile(
          api,
          [],
          new Map(),
          [],
          {
            fixedDelta: 0.001,
            gravity: [0, 0, 0],
            solverIterations: 100,
            frictionCone: "pyramidal",
            frictionImpedanceRatio: 1,
          },
          new Map([[body, mesh]]),
        );
        try {
          const { model: native, data } = simulation;
          const joint = api.mj_name2id(
            native,
            api.mjtObj.mjOBJ_JOINT.value,
            `${nodeName(mesh, 2)}j0`,
          );
          const qpos = at(native.jnt_qposadr, joint),
            dof = at(native.jnt_dofadr, joint);
          const length = at(data.flexvert_xpos, 7);
          const mu = youngModulus / (2 * (1 + poissonRatio));
          const lambda =
            (youngModulus * poissonRatio) /
            ((1 + poissonRatio) * (1 - 2 * poissonRatio));
          for (const [a, b, c] of [
            [0.01, 0.02, -0.03],
            [0, -0.8, 0],
          ] as const) {
            array(data.qpos).set([a * length, b * length, c * length], qpos);
            api.mj_forward(native, data);
            // F differs from identity only in column y; nodal force = P[:,y] * V/L.
            const strain = (a * a + (1 + b) ** 2 + c * c - 1) / 2;
            const stress = (lambda + 2 * mu) * strain;
            const expected =
              model === "neoHookean"
                ? [mu * a, (lambda + 2 * mu) * b, mu * c]
                : [a * (mu + stress), (1 + b) * stress, c * (mu + stress)];
            for (const [axis, value] of expected.entries())
              expect(-at(data.qfrc_passive, dof + axis)).toBeCloseTo(
                (value * length ** 2) / 6,
                8,
              );
          }
          array(data.qpos).fill(0);
          const velocity = 0.01 * length;
          array(data.qvel)[dof + 1] = velocity;
          api.mj_forward(native, data);
          // At rest, stiffness-proportional damping is -tau * K * velocity.
          const dampingForce =
            ((0.002 * (lambda + 2 * mu) * length) / 6) * velocity;
          expect(-at(data.qfrc_damper, dof + 1) / dampingForce).toBeCloseTo(
            1,
            4,
          );
        } finally {
          simulation.free();
        }
      }
  },
);

test("rejects dimensionless damping for MuJoCo neoHookean instead of converting its units", async () => {
  scene.add(
    new SoftBody({
      shape: {
        kind: "volume",
        positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        tetrahedra: [0, 1, 2, 3],
      },
      material: {
        density: 1000,
        elasticity: {
          model: "neoHookean",
          youngModulus: 1000,
          poissonRatio: 0.3,
          dampingRatio: 0.1,
        },
      },
    }),
  );
  await expect(createWorld()).rejects.toThrow("does not support dampingRatio");
});
