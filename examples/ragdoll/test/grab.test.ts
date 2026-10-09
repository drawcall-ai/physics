import { expect, it } from "vitest";
import { Scene, Vector3 } from "three";
import { RigidBody } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import { createRagdoll, simulationOptions } from "../model";
import { grab } from "../grab";

it.each([
  { backend: "Rapier", build: buildRapier },
  { backend: "MuJoCo", build: buildMujoco },
])(
  "$backend: lifts the ragdoll by its head, lets the floor stop a pull through it, and resets",
  async ({ build }) => {
    const world = await build({ scene: new Scene(), ...simulationOptions });
    try {
      const ragdoll = createRagdoll();
      world.scene.add(ragdoll);
      const head = ragdoll.getObjectByName("Head");
      const pelvis = ragdoll.getObjectByName("Pelvis");
      if (!(head instanceof RigidBody) || !(pelvis instanceof RigidBody))
        throw new Error("Missing ragdoll bodies");
      for (let frame = 0; frame < 120; frame++) world.update(1 / 60);
      expect(pelvis.position.y).toBeLessThan(0.3);
      const held = grab(ragdoll, head, head.getWorldPosition(new Vector3()));
      const start = head.position.y;
      for (let frame = 0; frame < 180; frame++) {
        held.move(new Vector3(0, start + (2 * frame) / 180, 0));
        world.update(1 / 60);
      }
      expect(head.position.y).toBeGreaterThan(1.2);
      expect(pelvis.position.y).toBeGreaterThan(0.4);
      for (let frame = 0; frame < 180; frame++) {
        held.move(new Vector3(0, -1, 0));
        world.update(1 / 60);
      }
      expect(head.position.y).toBeGreaterThan(0.1);
      held.release();
      expect(ragdoll.getObjectByName("Hand")).toBeUndefined();
      for (let frame = 0; frame < 60; frame++) world.update(1 / 60);
      expect(head.position.y).toBeLessThan(0.4);
      world.reset();
      expect(pelvis.position.y).toBeCloseTo(1.8);
    } finally {
      world.dispose();
    }
  },
  60000,
);
