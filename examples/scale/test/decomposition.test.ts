import { afterEach, expect, test } from "vitest";
import type { PhysicsWorld } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { Scene, Vector3 } from "three";
import { decomposition } from "../decomposition";

const worlds: PhysicsWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

test.each([
  { backend: "MuJoCo", build: buildWorld },
  { backend: "Rapier", build: buildRapier },
])(
  "$backend: the example preserves the opening on the right, including replay",
  async ({ build }) => {
    const { root, lanes } = decomposition();
    const world = await build({ scene: new Scene().add(root) });
    worlds.push(world);
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

test("the right frame needs world.decompose when added after building", async () => {
  const world = await buildWorld({ scene: new Scene() });
  worlds.push(world);
  const { root, lanes } = decomposition();
  world.scene.add(root);
  expect(() => world.update(world.fixedDelta)).toThrow(
    "await world.decompose(object)",
  );
  await world.decompose(root);
  for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
  const [left, right] = lanes;
  expect(left?.cube.position.y).toBeCloseTo(2.45, 2);
  expect(right?.cube.position.y).toBeCloseTo(0.25, 2);
}, 30000);
