import { Box3, BufferAttribute, Vector3 } from "three";
import { binary } from "../../generated/assets.js";

interface Gmsh {
  initialize(readConfigFiles: boolean, run: boolean): void;
  finalize(): void;
  option: { setNumber(name: string, value: number): void };
  model: {
    add(name: string): void;
    addDiscreteEntity(dimension: number): number;
    getEntities(dimension: number): { dimTags: number[] };
    mesh: {
      addNodes(
        dimension: number,
        tag: number,
        tags: number[],
        positions: number[],
      ): void;
      addElementsByType(
        tag: number,
        type: number,
        tags: number[],
        nodes: number[],
      ): void;
      classifySurfaces(
        angle: number,
        boundary: boolean,
        reparametrize: boolean,
        curveAngle: number,
      ): void;
      createGeometry(): void;
      generate(dimension: number): void;
      getNodes(): { nodeTags: number[]; coord: number[] };
      getElementsByType(type: number): { nodeTags: number[] };
    };
    geo: {
      addSurfaceLoop(tags: number[]): number;
      addVolume(shells: number[]): number;
      synchronize(): void;
    };
  };
}

function property(value: unknown, key: string): unknown {
  if (typeof value !== "object" || value === null) return undefined;
  return Reflect.get(value, key);
}

function isGmsh(value: unknown): value is Gmsh {
  return [
    "initialize",
    "finalize",
    "option.setNumber",
    "model.add",
    "model.addDiscreteEntity",
    "model.getEntities",
    "model.mesh.addNodes",
    "model.mesh.addElementsByType",
    "model.mesh.classifySurfaces",
    "model.mesh.createGeometry",
    "model.mesh.generate",
    "model.mesh.getNodes",
    "model.mesh.getElementsByType",
    "model.geo.addSurfaceLoop",
    "model.geo.addVolume",
    "model.geo.synchronize",
  ].every(
    (path) =>
      typeof path.split(".").reduce<unknown>(property, value) === "function",
  );
}

let core: Promise<Gmsh> | undefined;
async function load() {
  const module = await import("../../generated/gmsh/gmsh.mjs");
  const gmsh = await module.initialize({
    wasmBinary: binary("gmsh-core.wasm"),
  });
  if (!isGmsh(gmsh))
    throw new Error("FEM Gmsh asset is missing required meshing functions");
  return gmsh;
}

export async function tetrahedralize(
  positions: ArrayLike<number>,
  triangles: ArrayLike<number>,
  cellSize: number,
) {
  const bounds = new Box3().setFromBufferAttribute(
    new BufferAttribute(new Float64Array(positions), 3),
  );
  const center = bounds.getCenter(new Vector3());
  const extent = bounds.getSize(new Vector3());
  const size = Math.max(extent.x, extent.y, extent.z);
  if (!Number.isFinite(size) || size <= 0)
    throw new Error("FEM surface must have positive extent");
  if (!Number.isFinite(cellSize) || cellSize <= 0)
    throw new Error("FEM cellSize must be positive and finite");

  core ??= load().catch((error) => {
    core = undefined;
    throw error;
  });
  const gmsh = await core;
  // Gmsh's geometric tolerances should not depend on the object's physical scale.
  const points = Array.from(
    positions,
    (value, i) => (value - center.getComponent(i % 3)) / size,
  );
  gmsh.initialize(false, false);
  try {
    gmsh.option.setNumber("General.Terminal", 0);
    gmsh.option.setNumber("General.NumThreads", 1);
    gmsh.model.add("fem");
    const surface = gmsh.model.addDiscreteEntity(2);
    gmsh.model.mesh.addNodes(
      2,
      surface,
      Array.from({ length: points.length / 3 }, (_, i) => i + 1),
      points,
    );
    gmsh.model.mesh.addElementsByType(
      surface,
      2,
      [],
      Array.from(triangles, (value) => value + 1),
    );
    gmsh.model.mesh.classifySurfaces(Math.PI / 4, true, true, Math.PI);
    gmsh.model.mesh.createGeometry();
    const surfaces = gmsh.model
      .getEntities(2)
      .dimTags.filter((_, i) => i % 2 === 1);
    const loop = gmsh.model.geo.addSurfaceLoop(surfaces);
    gmsh.model.geo.addVolume([loop]);
    gmsh.model.geo.synchronize();
    gmsh.option.setNumber("Mesh.MeshSizeMin", cellSize / size);
    gmsh.option.setNumber("Mesh.MeshSizeMax", cellSize / size);
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0);
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0);
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0);
    gmsh.model.mesh.generate(3);
    const nodes = gmsh.model.mesh.getNodes();
    const elements = gmsh.model.mesh.getElementsByType(4);
    if (elements.nodeTags.length === 0)
      throw new Error("FEM meshing produced no tetrahedra");
    const order = nodes.nodeTags
      .map((tag, index) => ({ tag, index }))
      .sort((a, b) => a.tag - b.tag);
    const indices = new Map<number, number>();
    const result = new Float64Array(nodes.coord.length);
    const point = new Vector3();
    order.forEach(({ tag, index }, next) => {
      indices.set(tag, next);
      point
        .fromArray(nodes.coord, index * 3)
        .multiplyScalar(size)
        .add(center)
        .toArray(result, next * 3);
    });
    const cells = Uint32Array.from(elements.nodeTags, (tag) => {
      const index = indices.get(tag);
      if (index === undefined)
        throw new Error("FEM mesher returned an unknown node tag");
      return index;
    });
    const origin = new Vector3(),
      first = new Vector3(),
      second = new Vector3(),
      third = new Vector3();
    for (let i = 0; i < cells.length; i += 4) {
      const [a, b, c, d] = cells.subarray(i, i + 4);
      if (
        a === undefined ||
        b === undefined ||
        c === undefined ||
        d === undefined
      ) {
        throw new Error("FEM mesher returned an incomplete tetrahedron");
      }
      origin.fromArray(result, a * 3);
      first.fromArray(result, b * 3).sub(origin);
      second.fromArray(result, c * 3).sub(origin);
      third.fromArray(result, d * 3).sub(origin);
      const determinant = first.dot(second.cross(third));
      if (!Number.isFinite(determinant) || determinant === 0)
        throw new Error("FEM mesh contains a degenerate tetrahedron");
      if (determinant < 0) {
        cells[i] = b;
        cells[i + 1] = a;
      }
    }
    return { positions: result, tetrahedra: cells };
  } finally {
    gmsh.finalize();
  }
}
