import type { ReducedModel, ReducedRegion, Vec3 } from "@drawcall/physics";
import { copySoftMaterial } from "@drawcall/physics/backend";

export function parse(value: unknown): ReducedModel {
  const data = record(value, "reduced FEM model");
  if (data.version !== 1)
    throw new Error("Unsupported reduced FEM model version");
  const material = record(data.material, "FEM material");
  const elasticity = record(material.elasticity, "FEM elasticity");
  const law = elasticity.model;
  if (
    law !== "linear" &&
    law !== "neoHookean" &&
    law !== "saintVenantKirchhoff"
  )
    throw new Error("Unsupported reduced FEM elasticity");
  const physical = copySoftMaterial({
    density: scalar(material.density, "density"),
    elasticity: {
      model: law,
      youngModulus: scalar(elasticity.youngModulus, "Young's modulus"),
      poissonRatio: scalar(elasticity.poissonRatio, "Poisson ratio"),
      dampingTime: optionalScalar(elasticity.dampingTime, "dampingTime"),
      ...(elasticity.dampingRatio === undefined
        ? {}
        : { dampingRatio: scalar(elasticity.dampingRatio, "dampingRatio") }),
    },
    staticFriction: optionalScalar(material.staticFriction, "static friction"),
    dynamicFriction: optionalScalar(
      material.dynamicFriction,
      "dynamic friction",
    ),
    restitution: optionalScalar(material.restitution, "restitution"),
    torsionalFriction: optionalScalar(
      material.torsionalFriction,
      "torsional friction",
    ),
    rollingFriction: optionalScalar(
      material.rollingFriction,
      "rolling friction",
    ),
  });
  if ("dampingRatio" in physical.elasticity && physical.elasticity.dampingRatio)
    throw new Error("Reduced FEM uses dampingTime, not dampingRatio");
  const positions = vector(data.positions, "FEM positions", 3);
  const renderPositions = vector(
    data.renderPositions,
    "FEM render positions",
    3,
  );
  const nodes = positions.length / 3;
  const triangles = indices(data.triangles, nodes, "FEM contact triangles", 3);
  const mount = indices(data.mount, nodes, "FEM mount");
  const coordinates = indices(data.coordinates, nodes, "FEM coordinates");
  if (
    new Set(mount).size !== mount.length ||
    new Set(coordinates).size !== coordinates.length
  )
    throw new Error("Reduced FEM node selections must be unique");
  const fixed = new Set(mount);
  if (coordinates.some((node) => fixed.has(node)))
    throw new Error("Reduced FEM mount and coordinates overlap");
  const retained = new Set([...mount, ...coordinates]);
  if (triangles.some((node) => !retained.has(node)))
    throw new Error("Reduced FEM contact triangle uses an eliminated node");
  const dofs = coordinates.length * 3;
  const basis = matrix(data.basis, positions.length, dofs, "FEM basis");
  const mass = matrix(data.mass, dofs + 3, dofs + 3, "FEM mass");
  const stiffness = matrix(data.stiffness, dofs, dofs, "FEM stiffness");
  symmetric(mass, dofs + 3, "FEM mass");
  symmetric(stiffness, dofs, "FEM stiffness");
  for (const node of mount)
    for (let axis = 0; axis < 3; axis++)
      for (let j = 0; j < dofs; j++)
        if (Math.abs(basis[(node * 3 + axis) * dofs + j] ?? Infinity) > 1e-9)
          throw new Error("Reduced FEM basis moves its fixed mount");
  for (const [i, node] of coordinates.entries())
    for (let axis = 0; axis < 3; axis++)
      for (let j = 0; j < dofs; j++)
        if (
          Math.abs(
            (basis[(node * 3 + axis) * dofs + j] ?? Infinity) -
              (j === i * 3 + axis ? 1 : 0),
          ) > 1e-9
        )
          throw new Error(
            "Reduced FEM basis must reproduce its retained coordinates",
          );
  let quadrature: ReducedModel["quadrature"];
  if (data.quadrature !== undefined) {
    const sample = record(data.quadrature, "FEM quadrature");
    const weights = vector(sample.weights, "FEM quadrature weights");
    if (weights.some((weight) => weight <= 0))
      throw new Error("FEM quadrature weights must be positive");
    quadrature = Object.freeze({
      weights,
      gradients: matrix(
        sample.gradients,
        weights.length * 9,
        dofs,
        "FEM quadrature gradients",
      ),
    });
  }
  if (law !== "linear" && !quadrature)
    throw new Error("Nonlinear reduced FEM requires material quadrature");
  let surfaceError: ReducedModel["surfaceError"];
  if (data.surfaceError !== undefined) {
    const errors = record(data.surfaceError, "FEM surface error");
    const contact = scalar(errors.contact, "contact surface error");
    const render = scalar(errors.render, "render surface error");
    if (contact < 0 || render < 0)
      throw new Error("Invalid reduced FEM surface error");
    surfaceError = Object.freeze({ contact, render });
  }
  return Object.freeze({
    dofs,
    material: physical,
    positions,
    triangles,
    mount,
    coordinates,
    basis,
    mass,
    stiffness,
    gravity: matrix(data.gravity, dofs + 3, 3, "FEM gravity"),
    renderPositions,
    renderTriangles: indices(
      data.renderTriangles,
      renderPositions.length / 3,
      "FEM render triangles",
      3,
    ),
    renderBasis: matrix(
      data.renderBasis,
      renderPositions.length,
      dofs,
      "FEM render basis",
    ),
    contactRegion: region(data.contactRegion),
    mountRegion: region(data.mountRegion),
    quadrature,
    surfaceError,
  });
}

function record(value: unknown, label: string): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value))
    throw new Error(`Invalid ${label}`);
  return Object.fromEntries(Object.entries(value));
}
function scalar(value: unknown, label: string): number {
  if (typeof value !== "number" || !Number.isFinite(value))
    throw new Error(`Invalid FEM ${label}`);
  return value;
}
function optionalScalar(value: unknown, label: string): number | undefined {
  return value === undefined ? undefined : scalar(value, label);
}
function vector(value: unknown, label: string, stride = 1): readonly number[] {
  if (!Array.isArray(value) || !value.length || value.length % stride)
    throw new Error(`Invalid ${label}`);
  return Object.freeze(value.map((item: unknown) => scalar(item, label)));
}
function indices(
  value: unknown,
  count: number,
  label: string,
  stride = 1,
): readonly number[] {
  const values = vector(value, label, stride);
  if (values.some((v) => !Number.isInteger(v) || v < 0 || v >= count))
    throw new Error(`Invalid ${label} index`);
  return values;
}
function matrix(
  value: unknown,
  rows: number,
  cols: number,
  label: string,
): readonly number[] {
  const values = vector(value, label);
  if (values.length !== rows * cols)
    throw new Error(`Invalid ${label} dimensions`);
  return values;
}
function symmetric(
  values: readonly number[],
  size: number,
  label: string,
): void {
  for (let i = 0; i < size; i++)
    for (let j = 0; j < i; j++) {
      const a = values[i * size + j] ?? NaN,
        b = values[j * size + i] ?? NaN;
      if (Math.abs(a - b) > 1e-8 * Math.max(1, Math.abs(a), Math.abs(b)))
        throw new Error(`${label} must be symmetric`);
    }
}
function region(value: unknown): ReducedRegion {
  const data = record(value, "FEM region");
  const tuple = (value: unknown): Vec3 => {
    const values = vector(value, "FEM region bounds", 3);
    const [x, y, z] = values;
    if (
      values.length !== 3 ||
      x === undefined ||
      y === undefined ||
      z === undefined
    )
      throw new Error("FEM regions require three coordinates");
    return Object.freeze([x, y, z]);
  };
  const min = tuple(data.min),
    max = tuple(data.max);
  if (min.some((v, axis) => v > (max[axis] ?? -Infinity)))
    throw new Error("Invalid FEM region bounds");
  return Object.freeze({ min, max });
}
