import { afterEach, expect, test } from "vitest";
import { registry } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { Scene, Vector3 } from "three";
import { decomposition } from "../decomposition";

afterEach(() => {
  registry.world?.dispose();
  registry.clear();
});

test.each([
  { backend: "MuJoCo", build: buildWorld },
  { backend: "Rapier", build: buildRapier },
])(
  "$backend: the example preserves the opening on the right, including replay",
  async ({ build }) => {
    const { root, lanes } = decomposition();
    const world = await build(new Scene().add(root));
    for (let replay = 0; replay < 2; replay++) {
      for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
      const [left, right] = lanes;
      if (!left || !right) throw new Error("Missing comparison lanes");
      expect(left.cube.position.y).toBeCloseTo(2.45, 2);
      expect(right.cube.position.y).toBeCloseTo(0.25, 2);
      for (const { frame } of lanes) {
        expect(
          world.raycast(
            new Vector3(frame.position.x + 1.2, 3, 0),
            new Vector3(0, -1, 0),
            2,
          ),
        ).toMatchObject({ body: frame });
      }
      world.reset();
    }
  },
  30000,
);

test("the same right frame blocks the opening if added after building", async () => {
  const world = await buildWorld(new Scene());
  const { root, lanes } = decomposition();
  world.root.add(root);
  for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
  for (const { cube } of lanes) expect(cube.position.y).toBeCloseTo(2.45, 2);
});
