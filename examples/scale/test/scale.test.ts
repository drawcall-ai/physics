import { describe, expect, it } from "vitest";
import { Scene } from "three";
import { RigidBody } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import { cases } from "../cases";
import { specimen, verify } from "../specimen";

describe.each([
  { backend: "Rapier", build: buildRapier },
  { backend: "MuJoCo", build: buildMujoco },
])("$backend", ({ build }) => {
  it.each(cases)(
    "$name, including removal and recreation",
    async (spec) => {
      const world = await build({ scene: new Scene() });
      try {
        expect(await verify(world, spec)).toMatch(/^PASS:/);
        expect(await verify(world, spec)).toMatch(/^PASS:/);
      } finally {
        world.dispose();
      }
    },
    30000,
  );

  it("creates one triangle collider per transformed mesh child", async () => {
    const world = await build({ scene: new Scene() });
    const spec = cases.find(
      (spec) =>
        spec.kind === "triangle mesh" &&
        !spec.explicit &&
        spec.placement === "combined" &&
        !spec.error,
    );
    if (!spec) throw new Error("Missing compound triangle mesh case");
    const item = specimen(spec);
    await world.decompose(item.root);
    world.scene.add(item.root);
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
    const world = await build({ scene: new Scene() });
    const spec = cases.find(
      (spec) => spec.bodyType === "kinematic" && !spec.error,
    );
    if (!spec) throw new Error("Missing kinematic case");
    const item = specimen(spec);
    world.scene.add(item.root);
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

  it.each([1 / 60, 1 / 120])(
    "scaled kinematic lift maintains contact through full cycles at %s",
    async (fixedDelta) => {
      const spec = cases.find(
        (entry) => entry.name === "Kinematic lift: uniform ancestor scale",
      );
      if (!spec) throw new Error("Missing lift case");
      const item = specimen(spec, true);
      const world = await build({
        scene: new Scene().add(item.root),
        fixedDelta,
      });
      world.onAfterStep(item.step);
      const passenger = item.root.children.find(
        (child) => child instanceof RigidBody && child.bodyType === "dynamic",
      );
      if (!(passenger instanceof RigidBody))
        throw new Error("Missing lift passenger");
      let minimum = Infinity,
        maximum = -Infinity,
        speed = 0,
        slip = 0;
      try {
        for (let i = 0; i < 15 / fixedDelta; i++) {
          world.update(fixedDelta);
          if (world.time < 3) continue;
          minimum = Math.min(minimum, item.gap());
          maximum = Math.max(maximum, item.gap());
          const liftSpeed = item.target.getVelocity().linear.y;
          speed = Math.max(speed, Math.abs(liftSpeed));
          slip = Math.max(
            slip,
            Math.abs(passenger.getVelocity().linear.y - liftSpeed),
          );
        }
        expect(minimum).toBeGreaterThan(-0.002);
        expect(maximum).toBeLessThan(0.002);
        expect(speed).toBeGreaterThan(0.8);
        expect(slip).toBeLessThan(0.02);
        expect(maximum - minimum).toBeLessThan(0.001);
      } finally {
        item.dispose();
        world.dispose();
      }
    },
    30000,
  );

  it("drops and rotates one body with three automatically generated convex colliders", async () => {
    const world = await build({ scene: new Scene() });
    const spec = cases.find((spec) => spec.compound);
    if (!spec) throw new Error("Missing compound case");
    const item = specimen(spec, true);
    world.scene.add(item.root);
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
});
