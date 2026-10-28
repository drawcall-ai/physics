import {
  Matrix4,
  Mesh,
  Vector3,
  type BufferAttribute,
  type BufferGeometry,
  type InterleavedBufferAttribute,
  type Object3D,
} from "three";
import type { SoftBody } from "./body.js";
import { tube } from "./tube.js";

/** Backends supply world-space render vertices, or centerline nodes for a curve. */
export function updateSoftGeometry(
  body: SoftBody,
  positions: ArrayLike<number>,
): void {
  if (positions.length % 3)
    throw new Error("Incomplete soft-body render positions");
  body.updateWorldMatrix(true, false);
  const inverse = body.matrixWorld.clone().invert();
  const point = new Vector3();
  if (!("shape" in body.options))
    throw new Error("Reduced FEM needs reduced geometry reconstruction");
  const { shape } = body.options;
  if (shape.kind === "curve") {
    if (positions.length !== shape.points.length * 3)
      throw new Error("Curve topology changed");
    const points = Array.from({ length: positions.length / 3 }, (_, i) =>
      new Vector3().fromArray(positions, i * 3).applyMatrix4(inverse),
    );
    const geometry = tube(points, shape.radius);
    const target = body.geometry.getAttribute("position"),
      normal = body.geometry.getAttribute("normal");
    const source = geometry.getAttribute("position"),
      sourceNormal = geometry.getAttribute("normal");
    for (let i = 0; i < target.count; i++) {
      target.setXYZ(i, source.getX(i), source.getY(i), source.getZ(i));
      normal.setXYZ(
        i,
        sourceNormal.getX(i),
        sourceNormal.getY(i),
        sourceNormal.getZ(i),
      );
    }
    geometry.dispose();
    target.needsUpdate = true;
    normal.needsUpdate = true;
  } else {
    const attribute = body.geometry.getAttribute("position");
    if (positions.length !== attribute.count * 3)
      throw new Error("Soft-body render topology changed");
    for (let i = 0; i < attribute.count; i++) {
      point.fromArray(positions, i * 3).applyMatrix4(inverse);
      attribute.setXYZ(i, point.x, point.y, point.z);
    }
    attribute.needsUpdate = true;
    body.geometry.computeVertexNormals();
  }
  body.geometry.computeBoundingBox();
  body.geometry.computeBoundingSphere();
}

interface Skin {
  mesh: Mesh;
  geometry: BufferGeometry;
  vertices: readonly number[];
  position: BufferAttribute | InterleavedBufferAttribute;
  positionArray: ArrayLike<number>;
  positionVersion: number;
  index: BufferAttribute | null;
  indexArray: ArrayLike<number> | undefined;
  indexVersion: number | undefined;
  path: readonly Object3D[];
  children: readonly (readonly Object3D[])[];
  matrix: Matrix4;
  inverse: Matrix4;
}
const skins = new WeakMap<SoftBody, Skin[]>();

export function bindSoftGeometry(
  body: SoftBody,
  mesh: Mesh,
  vertices: ArrayLike<number>,
): void {
  const model = body.model;
  if (!model) throw new Error("Geometry binding requires a reduced FEM model");
  const geometry = mesh.geometry;
  const position = geometry.getAttribute("position");
  if (
    !position ||
    position.itemSize !== 3 ||
    position.count !== vertices.length
  )
    throw new Error("Reduced FEM visual binding has the wrong vertex count");
  const mapping = Array.from(vertices);
  if (
    mapping.some(
      (v) =>
        !Number.isInteger(v) || v < 0 || v >= model.renderPositions.length / 3,
    )
  )
    throw new Error("Reduced FEM visual binding has an invalid vertex");
  const { path, matrix } = frame(mesh, body);
  if (
    !matrix.elements.every(Number.isFinite) ||
    Math.abs(matrix.determinant()) < 1e-15
  )
    throw new Error("Invalid reduced FEM visual transform");
  const bindings = skins.get(body) ?? [];
  if (bindings.some((binding) => binding.geometry === geometry))
    throw new Error("Reduced FEM geometry is already bound");
  bindings.push({
    mesh,
    geometry,
    vertices: Object.freeze(mapping),
    position,
    positionArray: position.array,
    positionVersion: buffer(position).version,
    index: geometry.index,
    indexArray: geometry.index?.array,
    indexVersion: geometry.index?.version,
    path,
    children: path.map((node) => [...node.children]),
    matrix,
    inverse: matrix.clone().invert(),
  });
  skins.set(body, bindings);
}

export function copySoftGeometry(source: SoftBody, target: SoftBody): void {
  const bindings = skins.get(source);
  if (!bindings) return;
  for (const skin of bindings) validateSkin(skin, source);
  skins.delete(target);
  target.traverse((object) => {
    if (!(object instanceof Mesh)) return;
    const skin = bindings.find(
      (binding) => binding.geometry === object.geometry,
    );
    if (!skin) return;
    object.geometry = object.geometry.clone();
    bindSoftGeometry(target, object, skin.vertices);
  });
  if (target.model) updateReducedGeometry(target, target.model.renderPositions);
}

/** Writes reconstructed render nodes in the component's local frame. */
export function updateReducedGeometry(
  body: SoftBody,
  positions: ArrayLike<number>,
): void {
  if (!body.model || positions.length !== body.model.renderPositions.length)
    throw new Error("Reduced FEM render topology changed");
  if (!Array.from(positions).every(Number.isFinite))
    throw new Error("Non-finite reduced FEM geometry");
  const bindings = skins.get(body) ?? [];
  for (const skin of bindings) validateSkin(skin, body);
  writeGeometry(body.geometry, positions);
  for (const skin of bindings) {
    writeGeometry(skin.geometry, positions, skin);
    skin.positionVersion = buffer(skin.position).version;
  }
}

function writeGeometry(
  geometry: BufferGeometry,
  positions: ArrayLike<number>,
  skin?: Skin,
): void {
  const attribute = geometry.getAttribute("position");
  if (
    !attribute ||
    attribute.itemSize !== 3 ||
    attribute.count !== (skin ? skin.vertices.length : positions.length / 3)
  )
    throw new Error(
      "Reduced FEM render topology changed; reload the component",
    );
  const point = new Vector3();
  for (let i = 0; i < attribute.count; i++) {
    const vertex = skin ? skin.vertices[i] : i;
    if (vertex === undefined)
      throw new Error("Missing reduced FEM visual vertex");
    point.fromArray(positions, vertex * 3);
    if (skin) point.applyMatrix4(skin.inverse);
    attribute.setXYZ(i, point.x, point.y, point.z);
  }
  attribute.needsUpdate = true;
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
}

function frame(
  mesh: Mesh,
  body: SoftBody,
): { path: Object3D[]; matrix: Matrix4 } {
  const path: Object3D[] = [];
  const matrix = new Matrix4();
  let node: Object3D | null = mesh;
  while (node !== body) {
    if (!node)
      throw new Error(
        "Reduced FEM visual hierarchy changed; reload the component",
      );
    if (node.matrixAutoUpdate) node.updateMatrix();
    matrix.premultiply(node.matrix);
    path.push(node);
    node = node.parent;
  }
  return { path, matrix };
}

function buffer(attribute: BufferAttribute | InterleavedBufferAttribute) {
  return "data" in attribute ? attribute.data : attribute;
}

function validateSkin(skin: Skin, body: SoftBody): void {
  const { geometry, position, index } = skin;
  if (
    skin.mesh.geometry !== geometry ||
    geometry.getAttribute("position") !== position ||
    position.array !== skin.positionArray ||
    buffer(position).version !== skin.positionVersion ||
    geometry.index !== index ||
    index?.array !== skin.indexArray ||
    index?.version !== skin.indexVersion
  )
    throw new Error(
      "Reduced FEM visual geometry changed; reload the component",
    );
  const { path, matrix } = frame(skin.mesh, body);
  if (
    path.length !== skin.path.length ||
    path.some((node, i) => {
      const children = skin.children[i];
      return (
        node !== skin.path[i] ||
        !children ||
        node.children.length !== children.length ||
        node.children.some((child, j) => child !== children[j])
      );
    }) ||
    !matrix.equals(skin.matrix)
  )
    throw new Error(
      "Reduced FEM visual transform or hierarchy changed; reload the component",
    );
}
