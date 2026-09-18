import { expect, test } from "vitest";
import { setupWorld } from "@drawcall/physics-mujoco";
import { cases } from "../cases";
import { verify } from "../specimen";

test.each(cases)(
  "MuJoCo: $name",
  async (spec) => {
    const world = await setupWorld();
    try {
      expect(verify(world, spec)).toContain("PASS");
      expect(verify(world, spec)).toContain("PASS");
    } finally {
      world.dispose();
    }
  },
  30000,
);
