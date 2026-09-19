import { expect, it } from "vitest";
import { Vector3 } from "three";
import { RigidBody } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-rapier";
import { createRagdoll, simulationOptions } from "../model";
import { grab } from "../grab";

it("lifts the ragdoll by its head and lets the floor stop a pull through it", async () => {
  const world = await buildWorld(simulationOptions);
  try {
    const scene = createRagdoll();
    const head = scene.getObjectByName("Head");
    const pelvis = scene.getObjectByName("Pelvis");
    if (!(head instanceof RigidBody) || !(pelvis instanceof RigidBody))
      throw new Error("Missing ragdoll bodies");
    for (let frame = 0; frame < 120; frame++) world.update(1 / 60);
    expect(pelvis.position.y).toBeLessThan(0.3);
    const held = grab(scene, head, head.getWorldPosition(new Vector3()));
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
    expect(scene.getObjectByName("Hand")).toBeUndefined();
    for (let frame = 0; frame < 60; frame++) world.update(1 / 60);
    expect(head.position.y).toBeLessThan(0.4);
  } finally {
    world.dispose();
  }
});
