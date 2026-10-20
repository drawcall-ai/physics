import type * as Rapier from "@dimforge/rapier3d-compat";
import { splitTransform, type SoftBody } from "@drawcall/physics";
import { surfaceMesh } from "@drawcall/physics/backend";
import { Matrix4, Vector3 } from "three";

export function shape(api: typeof Rapier, body: SoftBody, transform: Matrix4) {
  const shape = body.options.shape;
  const elasticity = body.material.elasticity;
  const stretch =
    elasticity.model === "spring" && elasticity.stretch.frequency > 0;
  const bend =
    elasticity.model === "spring" && (elasticity.bend?.frequency ?? 0) > 0;
  const scale = splitTransform(transform).scale.x;
  if (shape.kind === "curve") {
    const positions = new Float32Array(
      shape.points.flatMap((point) => [...point]),
    );
    const edges: number[] = [],
      bends: number[] = [];
    const volumes = new Float32Array(shape.points.length);
    let length = 0;
    transformPositions(positions, transform);
    for (let i = 1; i < shape.points.length; i++) {
      edges.push(i - 1, i);
      if (i > 1) bends.push(i - 2, i);
      const segment = point(positions, i).distanceTo(point(positions, i - 1));
      length += segment;
      const share = (segment * Math.PI * (shape.radius * scale) ** 2) / 2;
      addVolume(volumes, i - 1, share);
      addVolume(volumes, i, share);
    }
    return {
      desc: new api.SoftBodyDesc(positions)
        .setEdges(stretch ? edges : [])
        .setBendEdges(bend ? bends : [])
        .setWire(edges)
        .setParticleRadius(shape.radius * scale),
      volume: length * Math.PI * (shape.radius * scale) ** 2,
      volumes,
      vertices: undefined,
      skin: false,
    };
  }
  if (shape.kind === "volume" && "tetrahedra" in shape) {
    const positions = new Float32Array(shape.positions);
    transformPositions(positions, transform);
    const cells = Array.from(shape.tetrahedra);
    const surface = body.geometry.getIndex();
    if (!surface) throw new Error("Soft volume has no boundary triangles");
    let volume = 0;
    const volumes = new Float32Array(positions.length / 3);
    for (let i = 0; i < cells.length; i += 4) {
      const [a, b, c, d] = cells.slice(i, i + 4);
      if (
        a === undefined ||
        b === undefined ||
        c === undefined ||
        d === undefined
      )
        throw new Error("Incomplete tetrahedron");
      const origin = point(positions, a);
      const cellVolume =
        Math.abs(
          point(positions, b)
            .sub(origin)
            .dot(
              point(positions, c)
                .sub(origin)
                .cross(point(positions, d).sub(origin)),
            ),
        ) / 6;
      volume += cellVolume;
      for (const node of [a, b, c, d]) addVolume(volumes, node, cellVolume / 4);
    }
    return {
      desc: new api.SoftBodyDesc(positions)
        .setCells(cells)
        .setSurface(new Uint32Array(surface.array))
        .setParticleRadius(0),
      volume,
      volumes,
      vertices: undefined,
      skin: false,
    };
  }
  const mesh = surfaceMesh(shape.geometry);
  transformPositions(mesh.positions, transform);
  if (shape.kind === "surface") {
    const desc = api.SoftBodyDesc.trimesh(mesh.positions, mesh.triangles);
    if (!desc) throw new Error("Rapier could not build the soft surface");
    // Zero-frequency Rapier constraints still solve velocity, and zero damping produces NaN.
    if (!stretch) desc.setEdges([]);
    if (!bend) desc.setBendEdges([]).setDihedrals([]);
    let area = 0;
    const volumes = new Float32Array(mesh.positions.length / 3);
    for (let i = 0; i < mesh.triangles.length; i += 3) {
      const [a, b, c] = mesh.triangles.slice(i, i + 3);
      if (a === undefined || b === undefined || c === undefined)
        throw new Error("Incomplete triangle");
      const origin = point(mesh.positions, a);
      const triangleArea =
        point(mesh.positions, b)
          .sub(origin)
          .cross(point(mesh.positions, c).sub(origin))
          .length() / 2;
      area += triangleArea;
      const share = (triangleArea * shape.thickness * scale) / 3;
      for (const node of [a, b, c]) addVolume(volumes, node, share);
    }
    return {
      desc: desc
        .setShapeMatching(false)
        .setParticleRadius((shape.thickness * scale) / 2)
        .setOriented(false),
      volume: area * shape.thickness * scale,
      volumes,
      vertices: mesh.vertices,
      skin: false,
    };
  }
  const desc = api.SoftBodyDesc.volumetric(
    mesh.positions,
    mesh.triangles,
    shape.cellSize * scale,
    true,
  );
  if (!desc) throw new Error("Rapier could not mesh the soft volume");
  let volume = 0;
  for (let i = 0; i < mesh.triangles.length; i += 3) {
    const [a, b, c] = mesh.triangles.slice(i, i + 3);
    if (a === undefined || b === undefined || c === undefined)
      throw new Error("Incomplete triangle");
    volume +=
      point(mesh.positions, a).dot(
        point(mesh.positions, b).cross(point(mesh.positions, c)),
      ) / 6;
  }
  return {
    desc: desc.setSkinCollision(true).setParticleRadius(0),
    volume: Math.abs(volume),
    volumes: undefined,
    vertices: mesh.vertices,
    skin: true,
  };
}

export function addVolume(
  volumes: Float32Array,
  node: number,
  share: number,
): void {
  const volume = volumes[node];
  if (volume === undefined)
    throw new Error("Missing soft body particle volume");
  volumes[node] = volume + share;
}

export function point(positions: ArrayLike<number>, index: number): Vector3 {
  const x = positions[index * 3],
    y = positions[index * 3 + 1],
    z = positions[index * 3 + 2];
  if (x === undefined || y === undefined || z === undefined)
    throw new Error("Missing soft body particle");
  return new Vector3(x, y, z);
}

export function transformPositions(
  positions: Float32Array,
  transform: Matrix4,
): void {
  for (let i = 0; i < positions.length / 3; i++)
    point(positions, i)
      .applyMatrix4(transform)
      .toArray(positions, i * 3);
}
