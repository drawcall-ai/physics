import {
  BufferGeometry,
  Float32BufferAttribute,
  Matrix4,
  Mesh,
  Object3D,
  Vector3,
} from "three";
import { surfaceMesh } from "@drawcall/physics/backend";
import { splitTransform } from "@drawcall/physics";

export interface Skin {
  readonly mesh: Mesh;
  /** Source-surface node for each vertex of this mesh. */
  readonly vertices: Uint32Array;
}

export interface Source {
  readonly positions: readonly number[];
  readonly triangles: readonly number[];
  readonly visual: Object3D;
  readonly skins: readonly Skin[];
  readonly pose: Matrix4;
}

/** Snapshot rest geometry. Owned loader results retain their root identity for GLTS lifecycle tracking. */
export function source(object: Object3D, owned = false): Source {
  if (owned && object.parent)
    throw new Error(
      "FEMLoader asset handler returned an object already attached to a scene",
    );
  const visual = owned ? object : object.clone(true);
  const matrix = object.matrixAutoUpdate
    ? new Matrix4().compose(object.position, object.quaternion, object.scale)
    : object.matrix;
  const { pose, scale } = splitTransform(matrix, "FEMLoader source");
  visual.position.set(0, 0, 0);
  visual.quaternion.identity();
  visual.scale.copy(scale);
  visual.updateMatrix();
  visual.updateMatrixWorld(true);
  const points: number[] = [];
  const triangles: number[] = [];
  const meshes: { mesh: Mesh; start: number; count: number }[] = [];
  const point = new Vector3();
  visual.traverseVisible((node) => {
    if (
      flag(node, "isSkinnedMesh") ||
      flag(node, "isInstancedMesh") ||
      flag(node, "isBatchedMesh")
    )
      throw new Error(
        "FEMLoader requires ordinary meshes in their rest configuration",
      );
    if (!isMesh(node)) return;
    const geometry = node.geometry;
    if (Object.keys(geometry.morphAttributes).length)
      throw new Error("FEMLoader does not accept morph targets");
    const positions = geometry.getAttribute("position");
    if (!positions || positions.itemSize !== 3)
      throw new Error("FEMLoader requires three-dimensional mesh positions");
    const count = geometry.index?.count ?? positions.count;
    if (
      geometry.drawRange.start !== 0 ||
      (geometry.drawRange.count !== Infinity &&
        geometry.drawRange.count !== count)
    )
      throw new Error("FEMLoader does not accept partial mesh draw ranges");
    if (
      !Number.isFinite(node.matrixWorld.determinant()) ||
      node.matrixWorld.determinant() <= 0
    )
      throw new Error("FEMLoader requires finite, positive mesh transforms");
    const start = points.length / 3;
    for (let i = 0; i < positions.count; i++) {
      point.fromBufferAttribute(positions, i).applyMatrix4(node.matrixWorld);
      if (![point.x, point.y, point.z].every(Number.isFinite))
        throw new Error("FEMLoader requires finite mesh positions");
      points.push(point.x, point.y, point.z);
    }
    for (let i = 0; i < count; i++) {
      const vertex = geometry.index?.getX(i) ?? i;
      if (!Number.isInteger(vertex) || vertex < 0 || vertex >= positions.count)
        throw new Error("FEMLoader mesh index is out of range");
      triangles.push(start + vertex);
    }
    node.geometry = geometry.clone();
    meshes.push({ mesh: node, start, count: positions.count });
  });
  if (!meshes.length)
    throw new Error("FEMLoader source contains no visible meshes");
  const merged = new BufferGeometry();
  merged.setAttribute("position", new Float32BufferAttribute(points, 3));
  merged.setIndex(triangles);
  const welded = surfaceMesh(merged);
  merged.dispose();
  return {
    positions: Array.from(welded.positions),
    triangles: Array.from(welded.triangles),
    visual,
    pose,
    skins: meshes.map(({ mesh, start, count }) => ({
      mesh,
      vertices: welded.vertices.slice(start, start + count),
    })),
  };
}

function flag(object: unknown, name: string): boolean {
  return (
    typeof object === "object" &&
    object !== null &&
    Reflect.get(object, name) === true
  );
}

function isMesh(object: Object3D): object is Mesh {
  return flag(object, "isMesh");
}

export function sceneOf(value: unknown): Object3D {
  if (isObject3D(value)) return value;
  const scene: unknown =
    typeof value === "object" && value !== null
      ? Reflect.get(value, "scene")
      : undefined;
  if (isObject3D(scene)) return scene;
  throw new Error("FEMLoader handler must return an Object3D or a glTF scene");
}

function isObject3D(value: unknown): value is Object3D {
  return flag(value, "isObject3D");
}
