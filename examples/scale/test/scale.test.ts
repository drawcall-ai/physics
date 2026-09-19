import { expect, it } from "vitest";
import { buildWorld } from "@drawcall/physics-rapier";
import { cases } from "../cases";
import { specimen, verify } from "../specimen";

it.each(cases)("$name, including removal and recreation", async (spec) => {
  const world = await buildWorld();
  try {
    expect(verify(world, spec)).toMatch(/^PASS:/);
    expect(verify(world, spec)).toMatch(/^PASS:/);
  } finally {
    world.dispose();
  }
});

it("creates one triangle collider per transformed mesh child", async () => {
  const world = await buildWorld();
  const spec = cases.find(
    (spec) =>
      spec.kind === "triangle mesh" &&
      !spec.explicit &&
      spec.placement === "combined" &&
      !spec.error,
  );
  if (!spec) throw new Error("Missing compound triangle mesh case");
  const item = specimen(spec);
  world.onAfterStep(item.step);
  try {
    expect(item.target.children.map((child) => child.type)).toEqual([
      "Mesh",
      "Mesh",
      "Mesh",
    ]);
    world.update(world.fixedDelta);
    expect(
      item.target.getColliders().map((collider) => collider.shape()),
    ).toMatchObject([
      { kind: "mesh", approximation: "trimesh" },
      { kind: "mesh", approximation: "trimesh" },
      { kind: "mesh", approximation: "trimesh" },
    ]);
    expect(item.boundsError()).toBeLessThan(1e-5);
  } finally {
    item.dispose();
    world.dispose();
  }
});

it("moves the scaled kinematic lift and carries its falling cube", async () => {
  const world = await buildWorld();
  const spec = cases.find((spec) => spec.type === "kinematic" && !spec.error);
  if (!spec) throw new Error("Missing kinematic case");
  const item = specimen(spec);
  world.onAfterStep(item.step);
  let low = Infinity;
  let high = -Infinity;
  try {
    for (let step = 0; step < 360; step++) {
      world.update(world.fixedDelta);
      low = Math.min(low, item.target.position.y);
      high = Math.max(high, item.target.position.y);
      if (step > 120) expect(Math.abs(item.gap())).toBeLessThan(0.06);
    }
    expect(high - low).toBeGreaterThan(1.2);
    expect(item.target.scale.toArray()).toEqual(spec.scale);
  } finally {
    item.dispose();
    world.dispose();
  }
});

it("drops and rotates one body with three automatically generated convex colliders", async () => {
  const world = await buildWorld();
  const spec = cases.find((spec) => spec.compound);
  if (!spec) throw new Error("Missing compound case");
  const item = specimen(spec, true);
  try {
    for (let step = 0; step < 30; step++) world.update(world.fixedDelta);
    expect(item.target.position.y).toBeLessThan(-0.5);
    expect(Math.abs(item.target.quaternion.w)).toBeLessThan(0.99);
    expect(
      item.target.getColliders().map((collider) => collider.shape()),
    ).toMatchObject([
      { kind: "mesh", approximation: "convexHull" },
      { kind: "mesh", approximation: "convexHull" },
      { kind: "mesh", approximation: "convexHull" },
    ]);
  } finally {
    item.dispose();
    world.dispose();
  }
});
