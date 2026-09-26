import { expect, it, vi } from "vitest";
import {
  BoxGeometry,
  BufferGeometry,
  Mesh,
  SphereGeometry,
  type BufferAttribute,
  Float32BufferAttribute,
  InterleavedBuffer,
  InterleavedBufferAttribute,
} from "three";
import { geometryVersion, snapshotGeometry } from "../src/geometry.js";
import { MeshCollider, RigidBody } from "../src/index.js";

it("snapshots interpreted positions, ignoring unrelated interleaved channels", () => {
  const data = new InterleavedBuffer(
    new Float32Array([1, 2, 3, 9, 4, 5, 6, 8, 7, 8, 9, 7]),
    4,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  expect(snapshotGeometry(geometry).positions).toEqual([
    1, 2, 3, 4, 5, 6, 7, 8, 9,
  ]);
});

it("changes version for edits marked with needsUpdate and replaced attributes, as three.js requires", () => {
  const position = new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0], 3);
  const geometry = new BufferGeometry().setAttribute("position", position);
  const versions = [geometryVersion(geometry)];
  position.setX(0, 2);
  versions.push(geometryVersion(geometry));
  position.needsUpdate = true;
  versions.push(geometryVersion(geometry));
  geometry.setIndex([0, 2, 1]);
  versions.push(geometryVersion(geometry));
  geometry.setIndex(null);
  versions.push(geometryVersion(geometry));
  geometry.setAttribute("position", position.clone());
  versions.push(geometryVersion(geometry));
  // Unmarked edits go unseen; removing the index restores the unindexed version.
  expect(versions[1]).toBe(versions[0]);
  expect(versions[4]).toBe(versions[2]);
  expect(new Set(versions).size).toBe(4);
});

it("changes version when an interleaved buffer is marked edited", () => {
  const data = new InterleavedBuffer(
    new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    3,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  const version = geometryVersion(geometry);
  data.array[0] = 2;
  data.needsUpdate = true;
  expect(geometryVersion(geometry)).not.toBe(version);
});

it("derives automatic shapes per body type and follows swapped geometry", () => {
  const shared = new SphereGeometry(1, 8, 4).translate(1, 0, 0);
  const bodies = (["static", "dynamic"] as const).map((type) => {
    const body = new RigidBody({ type });
    body.add(new Mesh(shared));
    return body;
  });
  expect(bodies.map((body) => body.getColliders()[0]!.shape())).toMatchObject([
    { kind: "mesh", approximation: "trimesh" },
    { kind: "mesh", approximation: "convexHull" },
  ]);
  const mesh = bodies[1]!.children[0] as Mesh;
  mesh.geometry = new BoxGeometry();
  expect(bodies[1]!.getColliders()[0]!.shape()).toMatchObject({ kind: "box" });
});

it("validates a geometry separately for each approximation", () => {
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], 3),
  );
  const collides = (approximation: "convexHull" | "trimesh") => {
    const body = new RigidBody({ type: "static" });
    body.add(new MeshCollider({ approximation }).setGeometry(geometry));
    return body.getColliders().map((collider) => collider.shape());
  };
  expect(() => collides("convexHull")).not.toThrow();
  expect(() => collides("trimesh")).toThrow("complete triangles");
});

it("reads no vertices again until the geometry is marked edited", () => {
  const detailed = new SphereGeometry(1, 32, 16);
  const auto = new BoxGeometry();
  const explicit = new RigidBody({ type: "static" });
  explicit.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(detailed),
  );
  const implicit = new RigidBody({ type: "static" });
  implicit.add(new Mesh(auto));
  const shapes = () =>
    [explicit, implicit].map((body) => body.getColliders()[0]!.shape());
  shapes();
  const read = vi.spyOn(
    detailed.getAttribute("position") as BufferAttribute,
    "getX",
  );
  // An unmarked edit that makes the box no longer a box goes unseen.
  const position = auto.getAttribute("position");
  position.setX(0, 3);
  for (let i = 0; i < 10; i++)
    expect(shapes()[1]).toMatchObject({ kind: "box" });
  expect(read).not.toHaveBeenCalled();
  position.needsUpdate = true;
  expect(shapes()[1]).not.toMatchObject({ kind: "box" });
});
