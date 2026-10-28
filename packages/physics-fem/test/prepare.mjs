import assert from "node:assert/strict";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { pathToFileURL } from "node:url";
import { GLTSLoader } from "@drawcall/glts";
import { join } from "node:path";
import test from "node:test";
import { BoxGeometry, CylinderGeometry, LoadingManager, Mesh } from "three";
import { FEMLoader } from "../dist/index.js";

const material = {
  density: 1000,
  elasticity: { model: "neoHookean", youngModulus: 1e6, poissonRatio: 0 },
};
const options = {
  material,
  mount: { min: [-0.011, -0.006, -0.006], max: [-0.0099, 0.006, 0.006] },
  contact: { min: [0.0099, -0.006, -0.006], max: [0.011, 0.006, 0.006] },
  mesh: { cellSize: 0.006, maxDofs: 24 },
};

function close(actual, expected, tolerance, label) {
  assert.ok(
    Math.abs(actual - expected) <= tolerance,
    `${label}: expected ${expected}, got ${actual}`,
  );
}

test(
  "WASM preparation recovers from invalid mounting and reproduces a P1/P2 clamped bar",
  { timeout: 180000 },
  async (t) => {
    await mkdir(join(process.cwd(), ".cache"), { recursive: true });
    const directory = await mkdtemp(
      join(process.cwd(), ".cache", "fem-prepare-"),
    );
    t.after(() => rm(directory, { recursive: true, force: true }));
    const cache = await mkdtemp(join(tmpdir(), "fem-prepare-"));
    t.after(() => rm(cache, { recursive: true, force: true }));
    const loader = new FEMLoader().setCachePath(cache);
    const url = pathToFileURL(join(directory, "fixture.glts"));
    await writeFile(
      url,
      `
      import { scene } from "@drawcall/glts";
      import { prepare } from "native";
      scene.userData.body = await prepare();
    `,
    );
    async function prepare(object, options) {
      let result;
      const host = new GLTSLoader(new LoadingManager(), {
        modules: {
          native: {
            prepare: () => (result = loader.loadAsync({ object, ...options })),
          },
        },
      });
      try {
        return (await host.loadAsync(url)).userData.body;
      } catch (error) {
        if (result) await result;
        throw error;
      } finally {
        host.dispose();
      }
    }
    const object = new Mesh(new BoxGeometry(0.02, 0.01, 0.01));
    await assert.rejects(
      prepare(object, {
        ...options,
        mount: { min: [1, 1, 1], max: [2, 2, 2] },
      }),
      /mount must select at least three non-collinear nodes/,
    );
    await assert.rejects(
      prepare(object, {
        ...options,
        mesh: { ...options.mesh, maxDofs: 9 },
      }),
      /Contact surface cannot fit maxDofs/,
    );
    await assert.rejects(
      prepare(new Mesh(new CylinderGeometry(0.01, 0.01, 0.02, 16)), {
        ...options,
        mount: { min: [-0.02, -0.0101, -0.02], max: [0.02, -0.0099, 0.02] },
        contact: { min: [-0.02, -0.0099, -0.02], max: [0.02, 0.02, 0.02] },
        mesh: { ...options.mesh, order: 1, maxDofs: 51 },
      }),
      /contact skin deviates by more than half cellSize/,
    );

    for (const order of [1, 2]) {
      const body = await prepare(object, {
        ...options,
        mesh: { ...options.mesh, order },
      });
      const model = body.model;
      assert.ok(model);
      const coordinates = [0, 1, 2].map((axis) =>
        model.renderPositions.filter((_, index) => index % 3 === axis),
      );
      const lower = coordinates.map((values) => Math.min(...values));
      const extent = coordinates.map(
        (values, axis) => Math.max(...values) - lower[axis],
      );
      const area = extent[1] * extent[2];
      const mass = material.density * extent[0] * area;
      const n = model.dofs;
      for (let axis = 0; axis < 3; axis++) {
        close(
          model.mass[axis * (n + 3) + axis],
          mass,
          mass * 1e-11,
          `P${order} total mass`,
        );
      }

      const displacement = Array(n).fill(0);
      for (const [i, node] of model.coordinates.entries()) {
        displacement[i * 3] =
          (1e-6 * (model.positions[node * 3] - lower[0])) / extent[0];
      }
      const force = [0, 0, 0];
      for (let i = 0; i < n; i++) {
        for (let j = 0; j < n; j++) {
          force[i % 3] += model.stiffness[i * n + j] * displacement[j];
        }
      }
      close(
        force[0],
        ((material.elasticity.youngModulus * area) / extent[0]) * 1e-6,
        1e-12,
        `P${order} axial force`,
      );
      close(force[1], 0, 1e-12, `P${order} transverse force Y`);
      close(force[2], 0, 1e-12, `P${order} transverse force Z`);
      for (let row = 0; row < model.positions.length; row++) {
        let value = 0;
        for (let j = 0; j < n; j++)
          value += model.basis[row * n + j] * displacement[j];
        const expected =
          row % 3 === 0
            ? (1e-6 * (model.positions[row] - lower[0])) / extent[0]
            : 0;
        close(value, expected, 1e-14, `P${order} displacement row ${row}`);
      }

      // At rest and nu=0, the nonlinear energy Hessian is mu*(G:G + G:G^T).
      const quadrature = model.quadrature;
      assert.ok(quadrature);
      const tangent = new Float64Array(n * n);
      const mu = material.elasticity.youngModulus / 2;
      for (const [sample, weight] of quadrature.weights.entries()) {
        assert.ok(weight > 0);
        const start = sample * 9 * n;
        for (let component = 0; component < 9; component++) {
          const transpose = (component % 3) * 3 + Math.floor(component / 3);
          for (let i = 0; i < n; i++) {
            const gradient =
              weight * mu * quadrature.gradients[start + component * n + i];
            for (let j = 0; j < n; j++) {
              tangent[i * n + j] +=
                gradient *
                (quadrature.gradients[start + component * n + j] +
                  quadrature.gradients[start + transpose * n + j]);
            }
          }
        }
      }
      for (let i = 0; i < tangent.length; i++) {
        close(
          tangent[i],
          model.stiffness[i],
          1e-8,
          `P${order} material tangent ${i}`,
        );
      }
      assert.ok(model.surfaceError.contact < 1e-12);
    }
  },
);
