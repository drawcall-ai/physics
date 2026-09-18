import { expect, test } from "vitest";
import { MeshCollider, RigidBody } from "@drawcall/physics";
import { BufferGeometry, Float32BufferAttribute, Vector3 } from "three";
import { heightfield } from "../src/model/heightfield.js";
import { setupWorld } from "../src/index.js";

function square(indices: number[]): BufferGeometry {
  return new BufferGeometry()
    .setAttribute(
      "position",
      new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1], 3),
    )
    .setIndex(indices);
}

test.each([
  [0, 2, 3, 0, 3, 1],
  [0, 2, 1, 1, 2, 3],
])("recognizes complete cells with either diagonal: %j", (...indices) => {
  const geometry = square(indices);
  try {
    expect(heightfield(geometry, "grid")).toBeDefined();
  } finally {
    geometry.dispose();
  }
});

test.each([
  [0, 2, 1, 0, 3, 1],
  [0, 0, 3, 0, 3, 1],
])("rejects incomplete or degenerate cells: %j", (...indices) => {
  const geometry = square(indices);
  try {
    expect(heightfield(geometry, "grid")).toBeUndefined();
  } finally {
    geometry.dispose();
  }
});

test("overlapping triangles preserve their uncovered area in raycasts", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const geometry = square([0, 2, 1, 0, 3, 1]);
  try {
    const body = new RigidBody({ type: "static" });
    body.add(
      new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry),
    );
    const down = new Vector3(0, -1, 0);
    expect(world.raycast(new Vector3(0.5, 1, 0.9), down, 2)).toBeNull();
    expect(
      world.raycast(new Vector3(0.5, 1, 0.1), down, 2)?.distance,
    ).toBeCloseTo(1);
  } finally {
    world.dispose();
    geometry.dispose();
  }
});
