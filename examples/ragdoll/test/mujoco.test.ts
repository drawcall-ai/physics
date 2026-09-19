import { expect, test } from "vitest";
import { RigidBody } from "@drawcall/physics";
import { Vector3 } from "three";
import { buildWorld } from "@drawcall/physics-mujoco";
import { createRagdoll, simulationOptions } from "../model";
import { grab } from "../grab";

test("MuJoCo ragdoll falls, can be grabbed and released, and resets", async () => {
  const world = await buildWorld(simulationOptions);
  try {
    const scene = createRagdoll();
    const pelvis = scene.getObjectByName("Pelvis");
    if (!(pelvis instanceof RigidBody)) throw new Error("Missing pelvis");
    for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
    expect(pelvis.position.y).toBeLessThan(0.4);
    const held = grab(scene, pelvis, pelvis.position.clone());
    held.move(new Vector3(0, 2, 0));
    for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
    expect(pelvis.position.y).toBeGreaterThan(1);
    held.release();
    for (let i = 0; i < 240; i++) world.update(world.fixedDelta);
    expect(pelvis.position.y).toBeLessThan(0.4);
    world.reset();
    expect(pelvis.position.y).toBeCloseTo(1.8);
  } finally {
    world.dispose();
  }
}, 60000);
