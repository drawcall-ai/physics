import type { ReducedModel } from "@drawcall/physics";

const quadratures = new WeakMap<
  ReducedModel,
  { weights: Float64Array; gradients: Float64Array }
>();

/**
 * Exact material evaluation on the stored deformation subspace; no element sampling.
 * neoHookean uses compressible log-J energy, unlike MuJoCo's native stable flex law.
 */
export function evaluateReduced(
  model: ReducedModel,
  displacement: ArrayLike<number>,
): {
  energy: number;
  force: Float64Array;
  stiffness: Float64Array;
} {
  const n = model.dofs;
  const q = Float64Array.from(displacement);
  if (q.length !== n || !q.every(Number.isFinite))
    throw new Error("Invalid reduced FEM displacement");
  const force = new Float64Array(n);
  const stiffness = new Float64Array(n * n);
  let energy = 0;
  const material = model.material.elasticity;
  if (material.model === "linear") {
    stiffness.set(model.stiffness);
    for (let i = 0; i < n; i++) {
      let value = 0;
      for (let j = 0; j < n; j++) value += at(stiffness, i * n + j) * at(q, j);
      force[i] = -value;
      energy += (at(q, i) * value) / 2;
    }
    if (!Number.isFinite(energy) || !force.every(Number.isFinite))
      throw new Error("Non-finite reduced FEM material response");
    return { energy, force, stiffness };
  }
  if (
    material.model !== "neoHookean" &&
    material.model !== "saintVenantKirchhoff"
  )
    throw new Error(`Unsupported reduced FEM material: ${material.model}`);
  const quadrature = compile(model);
  const mu = material.youngModulus / (2 * (1 + material.poissonRatio));
  const lambda =
    (material.youngModulus * material.poissonRatio) /
    ((1 + material.poissonRatio) * (1 - 2 * material.poissonRatio));
  const f = new Float64Array(9);
  const stress = new Float64Array(9);
  const tangent = new Float64Array(81);
  const product = new Float64Array(9 * n);
  for (const [sample, weight] of quadrature.weights.entries()) {
    const start = sample * 9 * n;
    for (let row = 0; row < 9; row++) {
      let value = row % 4 === 0 ? 1 : 0;
      for (let j = 0; j < n; j++)
        value += at(quadrature.gradients, start + row * n + j) * at(q, j);
      f[row] = value;
    }
    energy +=
      weight * materialResponse(f, mu, lambda, material.model, stress, tangent);
    for (let row = 0; row < 9; row++)
      for (let j = 0; j < n; j++) {
        let value = 0;
        for (let column = 0; column < 9; column++)
          value +=
            at(tangent, row * 9 + column) *
            at(quadrature.gradients, start + column * n + j);
        product[row * n + j] = value;
      }
    for (let i = 0; i < n; i++) {
      let gradient = 0;
      for (let row = 0; row < 9; row++)
        gradient +=
          at(quadrature.gradients, start + row * n + i) * at(stress, row);
      force[i] = at(force, i) - weight * gradient;
      for (let j = 0; j <= i; j++) {
        let value = 0;
        for (let row = 0; row < 9; row++)
          value +=
            at(quadrature.gradients, start + row * n + i) *
            at(product, row * n + j);
        stiffness[i * n + j] = at(stiffness, i * n + j) + weight * value;
      }
    }
  }
  for (let i = 0; i < n; i++)
    for (let j = 0; j < i; j++) stiffness[j * n + i] = at(stiffness, i * n + j);
  if (
    !Number.isFinite(energy) ||
    !force.every(Number.isFinite) ||
    !stiffness.every(Number.isFinite)
  )
    throw new Error("Non-finite reduced FEM material response");
  return { energy, force, stiffness };
}

function compile(model: ReducedModel): {
  weights: Float64Array;
  gradients: Float64Array;
} {
  const existing = quadratures.get(model);
  if (existing) return existing;
  if (!model.quadrature) throw new Error("Missing reduced FEM quadrature");
  // Frozen public arrays keep the model immutable. Private dense buffers avoid
  // generic array access inside the numerical kernel without exposing mutation.
  const value = {
    weights: Float64Array.from(model.quadrature.weights),
    gradients: Float64Array.from(model.quadrature.gradients),
  };
  quadratures.set(model, value);
  return value;
}

function at(values: Float64Array, index: number): number {
  const value = values[index];
  if (value === undefined) throw new Error("Incomplete reduced FEM array");
  return value;
}

function materialResponse(
  f: Float64Array,
  mu: number,
  lambda: number,
  law: "neoHookean" | "saintVenantKirchhoff",
  stress: Float64Array,
  tangent: Float64Array,
): number {
  const a = at(f, 0),
    b = at(f, 1),
    c = at(f, 2);
  const d = at(f, 3),
    e = at(f, 4),
    g = at(f, 5);
  const h = at(f, 6),
    i = at(f, 7),
    j = at(f, 8);
  const determinant =
    a * (e * j - g * i) - b * (d * j - g * h) + c * (d * i - e * h);
  if (!(determinant > 0)) throw new Error("Reduced FEM element inverted");
  if (law === "neoHookean") {
    // Compressible log-J Neo-Hookean: psi = mu/2*(F:F-3) - mu*ln(J) + lambda/2*ln(J)^2.
    const inverseTranspose = new Float64Array([
      e * j - g * i,
      g * h - d * j,
      d * i - e * h,
      c * i - b * j,
      a * j - c * h,
      b * h - a * i,
      b * g - c * e,
      c * d - a * g,
      a * e - b * d,
    ]);
    for (let row = 0; row < 9; row++)
      inverseTranspose[row] = at(inverseTranspose, row) / determinant;
    const log = Math.log(determinant),
      coefficient = lambda * log - mu;
    for (let row = 0; row < 9; row++) {
      stress[row] = mu * at(f, row) + coefficient * at(inverseTranspose, row);
      const x = Math.floor(row / 3),
        y = row % 3;
      for (let column = 0; column < 9; column++) {
        const u = Math.floor(column / 3),
          v = column % 3;
        tangent[row * 9 + column] =
          (row === column ? mu : 0) +
          lambda * at(inverseTranspose, row) * at(inverseTranspose, column) -
          coefficient *
            at(inverseTranspose, x * 3 + v) *
            at(inverseTranspose, u * 3 + y);
      }
    }
    return (
      (mu / 2) * (f.reduce((sum, value) => sum + value * value, 0) - 3) -
      mu * log +
      (lambda / 2) * log * log
    );
  }
  const strain = new Float64Array(9),
    second = new Float64Array(9);
  let trace = 0;
  for (let row = 0; row < 3; row++)
    for (let column = 0; column < 3; column++) {
      let value = row === column ? -1 : 0;
      for (let k = 0; k < 3; k++)
        value += at(f, k * 3 + row) * at(f, k * 3 + column);
      strain[row * 3 + column] = value / 2;
      if (row === column) trace += value / 2;
    }
  for (let row = 0; row < 9; row++)
    second[row] =
      2 * mu * at(strain, row) + (row % 4 === 0 ? lambda * trace : 0);
  for (let row = 0; row < 9; row++) {
    const x = Math.floor(row / 3),
      y = row % 3;
    let value = 0;
    for (let k = 0; k < 3; k++)
      value += at(f, x * 3 + k) * at(second, k * 3 + y);
    stress[row] = value;
    for (let column = 0; column < 9; column++) {
      const u = Math.floor(column / 3),
        v = column % 3;
      let metric = 0;
      for (let k = 0; k < 3; k++) metric += at(f, x * 3 + k) * at(f, u * 3 + k);
      tangent[row * 9 + column] =
        (x === u ? at(second, v * 3 + y) : 0) +
        lambda * at(f, row) * at(f, column) +
        mu * (at(f, x * 3 + v) * at(f, u * 3 + y) + (y === v ? metric : 0));
    }
  }
  return (
    mu * strain.reduce((sum, value) => sum + value * value, 0) +
    (lambda / 2) * trace * trace
  );
}
