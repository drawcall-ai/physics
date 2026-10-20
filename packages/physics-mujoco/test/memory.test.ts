import { expect, test } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { compile } from "../src/model/compile.js";
import { modelOptions } from "../src/options.js";
import { createWorld, scene } from "./fixtures.js";

test("passes the requested arena size to MuJoCo", async () => {
  const memory = 16 * 1024 ** 2;
  const options = modelOptions(
    { scene, memory },
    { fixedDelta: 0.001, gravity: [0, 0, 0], solverIterations: 50 },
  );
  const sim = compile(await loadMujoco(), [], new Map(), [], options);
  try {
    expect(sim.model.narena).toBe(memory);
  } finally {
    sim.free();
  }
});

test.each([0, -1, 1.5, NaN, Infinity])(
  "rejects invalid memory %s",
  async (memory) => {
    await expect(createWorld({ memory })).rejects.toThrow(
      "memory must be a positive integer number of bytes",
    );
  },
);
