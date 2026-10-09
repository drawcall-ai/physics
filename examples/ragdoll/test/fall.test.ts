import { expect, it } from "vitest";
import { Scene } from "three";
import { RigidBody } from "@drawcall/physics";
import { buildWorld as buildRapier } from "@drawcall/physics-rapier";
import { buildWorld as buildMujoco } from "@drawcall/physics-mujoco";
import { createRagdoll, simulationOptions } from "../model";

// MuJoCo's ragdoll measurably rebounds about 3 cm on impact.
const backends = [
  { backend: "Rapier", build: buildRapier, maxRebound: 0.01 },
  { backend: "MuJoCo", build: buildMujoco, maxRebound: 0.04 },
];

it.each(
  backends.flatMap((backend) =>
    [15, 30, 60].map((fps) => ({ ...backend, fps })),
  ),
)(
  "$backend: falls forward without rebounding upright at $fps FPS",
  async ({ build, fps, maxRebound }) => {
    const world = await build({ scene: new Scene(), ...simulationOptions });
    try {
      const ragdoll = createRagdoll();
      world.scene.add(ragdoll);
      const bodies = ragdoll.children.filter(
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
      expect(rebound).toBeLessThan(maxRebound);
      const pelvis = ragdoll.getObjectByName("Pelvis");
      expect(pelvis?.position.y).toBeLessThan(0.3);
      expect(pelvis?.position.z).toBeGreaterThan(1);
    } finally {
      world.dispose();
    }
  },
  60000,
);
