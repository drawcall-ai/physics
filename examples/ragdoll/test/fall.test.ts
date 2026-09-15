import { expect, it } from "vitest";
import { RigidBody } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import { createRagdoll, simulationOptions } from "../model";

it.each([15, 30, 60])(
  "falls forward without rebounding upright at %i FPS",
  async (fps) => {
    const world = await setupWorld(simulationOptions);
    try {
      const scene = createRagdoll();
      const bodies = scene.children.filter(
        (object): object is RigidBody =>
          object instanceof RigidBody && object.bodyType === "dynamic",
      );
      const mass = bodies.reduce(
        (sum, body) => sum + (body.options.mass ?? 0),
        0,
      );
      let low = Infinity,
        rebound = 0;
      for (let frame = 0; frame < 2 * fps; frame++) {
        world.update(1 / fps);
        if (frame < fps) {
          const height =
            bodies.reduce(
              (sum, body) => sum + body.position.y * (body.options.mass ?? 0),
              0,
            ) / mass;
          low = Math.min(low, height);
          rebound = Math.max(rebound, height - low);
        }
      }
      expect(rebound).toBeLessThan(0.01);
      const pelvis = scene.getObjectByName("Pelvis");
      expect(pelvis?.position.y).toBeLessThan(0.3);
      expect(pelvis?.position.z).toBeGreaterThan(1);
    } finally {
      world.dispose();
    }
  },
);
