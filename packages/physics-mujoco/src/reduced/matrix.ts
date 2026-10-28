/** Checked access also works for the immutable arrays in a reduced asset. */
export function value(values: ArrayLike<number>, index: number): number {
  const result = values[index];
  if (result === undefined || !Number.isFinite(result))
    throw new Error(`Invalid reduced FEM coefficient at ${index}`);
  return result;
}

/** Lower-triangular Cholesky factor of a positive definite, row-major matrix. */
export function cholesky(
  matrix: ArrayLike<number>,
  size: number,
): Float64Array {
  if (matrix.length !== size * size)
    throw new Error("Reduced FEM matrix size mismatch");
  const result = new Float64Array(matrix.length);
  for (let row = 0; row < size; row++) {
    for (let column = 0; column <= row; column++) {
      let diagonal = value(matrix, row * size + column);
      for (let k = 0; k < column; k++)
        diagonal -=
          value(result, row * size + k) * value(result, column * size + k);
      if (row === column) {
        if (!(diagonal > 0))
          throw new Error(
            "MuJoCo reduced FEM requires positive definite mass and elastic tangent matrices",
          );
        result[row * size + column] = Math.sqrt(diagonal);
      } else {
        result[row * size + column] =
          diagonal / value(result, column * size + column);
      }
    }
  }
  return result;
}

/** Absolute retained-vertex coordinates from a serial translation tree's joint coordinates. */
export function positions(joints: ArrayLike<number>): Float64Array {
  const result = Float64Array.from(joints);
  for (let i = 6; i < result.length; i++)
    result[i] = value(result, i) + value(result, i - 3);
  return result;
}

/** Transpose of positions(): physical forces become serial-joint generalized forces. */
export function forces(load: ArrayLike<number>): Float64Array {
  const result = Float64Array.from(load);
  for (let i = result.length - 4; i >= 3; i--)
    result[i] = value(result, i) + value(result, i + 3);
  return result;
}

/** Tᵀ A T, where T sums preceding XYZ translations after the mount's first three DOFs. */
export function serial(
  matrix: ArrayLike<number>,
  size: number,
  first = 3,
): Float64Array {
  const result = Float64Array.from(matrix);
  if (result.length !== size * size)
    throw new Error("Reduced FEM matrix size mismatch");
  for (let row = 0; row < size; row++)
    for (let column = size - 4; column >= first; column--)
      result[row * size + column] =
        value(result, row * size + column) +
        value(result, row * size + column + 3);
  for (let row = size - 4; row >= first; row--)
    for (let column = 0; column < size; column++)
      result[row * size + column] =
        value(result, row * size + column) +
        value(result, (row + 3) * size + column);
  return result;
}

export function multiply(
  matrix: ArrayLike<number>,
  vector: ArrayLike<number>,
): Float64Array {
  const result = new Float64Array(vector.length);
  if (matrix.length !== vector.length * vector.length)
    throw new Error("Reduced FEM matrix size mismatch");
  for (let row = 0; row < result.length; row++)
    for (let column = 0; column < result.length; column++)
      result[row] =
        value(result, row) +
        value(matrix, row * result.length + column) * value(vector, column);
  return result;
}
