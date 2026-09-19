import { expect, test } from "vitest";
import { MeshCollider, RigidBody, registry } from "@drawcall/physics";
import { BufferGeometry, Float32BufferAttribute, Vector3 } from "three";
import { heightfield } from "../src/model/heightfield.js";
import { buildWorld } from "../src/index.js";

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

test("rejects overlapping open triangles during initial mesh preparation", async () => {
  const geometry = square([0, 2, 1, 0, 3, 1]);
  const body = new RigidBody({ type: "static" });
  body.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry),
  );
  try {
    await expect(buildWorld()).rejects.toThrow("closed");
    expect(registry.world).toBeUndefined();
    expect(body.disposed).toBe(false);
  } finally {
    body.dispose();
    geometry.dispose();
  }
});

test("prepares native terrain before the first step", async () => {
  const geometry = square([0, 2, 1, 1, 2, 3]);
  new RigidBody({ type: "static" }).add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(geometry),
  );
  const world = await buildWorld();
  try {
    expect(world.time).toBe(0);
    expect(
      world.raycast(new Vector3(0.5, 1, 0.9), new Vector3(0, -1, 0), 2)
        ?.distance,
    ).toBeCloseTo(1);
  } finally {
    world.dispose();
    geometry.dispose();
  }
});
