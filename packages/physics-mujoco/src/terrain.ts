import { BufferGeometry, Matrix4, Vector3 } from "three";

/** A regular, complete height grid can use MuJoCo's native terrain collision instead of triangle seams. */
export function terrain(geometry: BufferGeometry, name: string) {
  const positions = geometry.getAttribute("position");
  const points = Array.from({ length: positions.count }, (_, i) =>
    new Vector3().fromBufferAttribute(positions, i),
  );
  const xs = [...new Set(points.map((p) => p.x))].sort((a, b) => a - b);
  const zs = [...new Set(points.map((p) => p.z))].sort((a, b) => a - b);
  if (xs.length < 2 || zs.length < 2 || !uniform(xs) || !uniform(zs))
    return undefined;
  const heights = new Map<string, number>();
  for (const p of points) {
    const key = `${p.x},${p.z}`;
    const previous = heights.get(key);
    if (previous !== undefined && previous !== p.y) return undefined;
    heights.set(key, p.y);
  }
  if (heights.size !== xs.length * zs.length) return undefined;
  const cells = new Map<string, Set<string>>();
  const indices = geometry.index;
  for (let i = 0; i < (indices?.count ?? points.length); i += 3) {
    const vertices = [0, 1, 2].map(
      (j) => points[indices ? indices.getX(i + j) : i + j],
    );
    if (vertices.some((p) => !p)) return undefined;
    const coordinates = vertices.flatMap((p) =>
      p ? [[xs.indexOf(p.x), zs.indexOf(p.z)]] : [],
    );
    const x = Math.min(...coordinates.map((p) => p[0] ?? Infinity));
    const z = Math.min(...coordinates.map((p) => p[1] ?? Infinity));
    if (
      Math.max(...coordinates.map((p) => p[0] ?? -Infinity)) !== x + 1 ||
      Math.max(...coordinates.map((p) => p[1] ?? -Infinity)) !== z + 1
    )
      return undefined;
    const key = `${x},${z}`;
    const cell = cells.get(key) ?? new Set<string>();
    cell.add(
      coordinates
        .map((p) => p.join(","))
        .sort()
        .join(";"),
    );
    cells.set(key, cell);
  }
  if (
    cells.size !== (xs.length - 1) * (zs.length - 1) ||
    [...cells.values()].some((c) => c.size !== 2)
  )
    return undefined;
  const values = zs.flatMap((z) =>
    xs.map((x) => {
      const value = heights.get(`${x},${z}`);
      if (value === undefined) throw new Error("Incomplete terrain grid");
      return value;
    }),
  );
  // Coplanar cells ensure the source and MuJoCo triangle diagonals describe the same surface.
  for (let z = 0; z < zs.length - 1; z++)
    for (let x = 0; x < xs.length - 1; x++) {
      const a = values[z * xs.length + x],
        b = values[z * xs.length + x + 1],
        c = values[(z + 1) * xs.length + x],
        d = values[(z + 1) * xs.length + x + 1];
      if (
        a === undefined ||
        b === undefined ||
        c === undefined ||
        d === undefined ||
        Math.abs(a + d - b - c) > 1e-6
      )
        return undefined;
    }
  const min = Math.min(...values),
    max = Math.max(...values);
  const firstX = xs[0],
    lastX = xs.at(-1),
    firstZ = zs[0],
    lastZ = zs.at(-1);
  if (
    firstX === undefined ||
    lastX === undefined ||
    firstZ === undefined ||
    lastZ === undefined
  )
    return undefined;
  return {
    asset: `<hfield name="${name}" nrow="${zs.length}" ncol="${xs.length}" size="${(lastX - firstX) / 2} ${(lastZ - firstZ) / 2} ${Math.max(max - min, 1e-6)} 0.001" elevation="${values.join(" ")}"/>`,
    matrix: new Matrix4()
      .makeTranslation((firstX + lastX) / 2, min, (firstZ + lastZ) / 2)
      .multiply(new Matrix4().makeRotationX(-Math.PI / 2)),
  };
}
function uniform(values: number[]): boolean {
  const first = values[0],
    last = values.at(-1);
  if (first === undefined || last === undefined) return false;
  const step = (last - first) / (values.length - 1);
  return values.every((v, i) => Math.abs(v - first - i * step) < 1e-6);
}
