import { RigidBody } from "@drawcall/physics";
import { expect, test } from "vitest";
import { buildWorld } from "@drawcall/physics-mujoco";
import { cases } from "../cases";
import { specimen, verify } from "../specimen";

test.each(cases)(
  "MuJoCo: $name",
  async (spec) => {
    const world = await buildWorld();
    try {
      expect(verify(world, spec)).toContain("PASS");
      expect(verify(world, spec)).toContain("PASS");
    } finally {
      world.dispose();
    }
  },
  30000,
);

test.each([1 / 60, 1 / 120])(
  "scaled kinematic lift maintains contact through full cycles at %s",
  async (fixedDelta) => {
    const spec = cases.find(
      (entry) => entry.name === "Kinematic lift: uniform ancestor scale",
    );
    if (!spec) throw new Error("Missing lift case");
    const item = specimen(spec, true);
    const world = await buildWorld({ fixedDelta });
    const stop = world.onAfterStep(item.step);
    const passenger = item.root.children.find(
      (child) => child instanceof RigidBody && child.bodyType === "dynamic",
    );
    let minimum = Infinity,
      maximum = -Infinity,
      speed = 0,
      slip = 0;
    try {
      if (!(passenger instanceof RigidBody))
        throw new Error("Missing lift passenger");
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
      stop();
      item.dispose();
      world.dispose();
    }
  },
);
