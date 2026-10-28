import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { PlaneGeometry, Scene, Vector3 } from "three";
import {
  RigidBody,
  SoftAttachment,
  SoftBody,
  SphereCollider,
} from "@drawcall/physics";
import { FEMLoader } from "@drawcall/physics-fem";
import { buildWorld } from "@drawcall/physics-mujoco";
import { expect, test } from "vitest";
import {
  density,
  length,
  thickness,
  width,
  youngModulus,
  theory,
} from "./beam.ts";

test("a reduced cantilever and native cloth settle together and preserve analytical force balance", async () => {
  const cache = await mkdtemp(join(tmpdir(), "fem-beam-"));
  const loader = new FEMLoader().setCachePath(cache);
  try {
    const beam = await loader.loadAsync({
      url: new URL("./beam.glts", import.meta.url).href,
      material: {
        density,
        elasticity: {
          model: "linear",
          youngModulus,
          poissonRatio: 0.3,
          dampingTime: 0.04,
        },
      },
      mount: {
        min: [-1e-4, -thickness, -width],
        max: [1e-4, thickness, width],
      },
      contact: {
        min: [length - 1e-6, -thickness * 0.49, -width * 0.49],
        max: [length + 1e-6, thickness * 0.49, width * 0.49],
      },
      mesh: { cellSize: 0.075, order: 2, maxDofs: 24 },
    });
    if (!beam.model) throw new Error("Missing reduced beam");
    beam.position.set(-1.5, 1, 0);
    const geometry = new PlaneGeometry(1, 1, 8, 8).rotateX(-Math.PI / 2);
    const cloth = new SoftBody({
      shape: { kind: "surface", geometry, thickness: 0.001 },
      material: {
        density: 300,
        dynamicFriction: 0.5,
        elasticity: {
          model: "saintVenantKirchhoff",
          youngModulus: 200000,
          poissonRatio: 0.3,
          dampingTime: 0.002,
        },
      },
      selfCollision: true,
    });
    geometry.dispose();
    cloth.position.y = 0.95;
    cloth.setLinearDamping(0.5);
    const sphere = new RigidBody({ bodyType: "static" });
    sphere.position.y = 0.47;
    sphere.add(new SphereCollider({ radius: 0.25 }));
    const scene = new Scene().add(
      beam,
      cloth,
      sphere,
      new SoftAttachment({
        body: beam,
        region: beam.model.mount,
        target: null,
      }),
      new SoftAttachment({ body: cloth, region: [0, 8], target: null }),
    );
    const world = await buildWorld({
      scene,
      fixedDelta: 1 / 500,
      solverIterations: 60,
    });
    const position = beam.geometry.getAttribute("position");
    const tip = Array.from({ length: position.count }, (_, i) => i).filter(
      (i) => position.getX(i) > length - 1e-6,
    );
    const deflection = () =>
      -tip.reduce((sum, i) => sum + position.getY(i), 0) / tip.length;
    const step = (seconds: number) => {
      for (let i = 0; i < seconds / world.fixedDelta; i++)
        world.update(world.fixedDelta);
    };
    let force = 0;
    const load = new Vector3();
    world.onBeforeStep(() => beam.applyForce(load.set(0, -force, 0)));
    try {
      step(20);
      const bodies = [beam, cloth];
      const settled = bodies.map((body) => body.getPositions());
      for (let sample = 0; sample < 50; sample++) {
        step(0.02);
        for (const [index, body] of bodies.entries()) {
          const initial = settled[index];
          if (!initial) throw new Error("Missing settled positions");
          expect(
            body.getPositions().every((value, i) => {
              const previous = initial[i];
              return (
                previous !== undefined && Math.abs(value - previous) < 0.001
              );
            }),
          ).toBe(true);
        }
      }
      const baseline = deflection();
      expect(Math.abs(baseline / theory(0, 9.81) - 1)).toBeLessThan(0.03);
      force = 40;
      step(5);
      expect(Math.abs(deflection() / theory(force, 9.81) - 1)).toBeLessThan(
        0.03,
      );
      force = 0;
      step(5);
      expect(Math.abs(deflection() - baseline)).toBeLessThan(1e-5);
      world.reset();
      expect(Math.abs(deflection())).toBeLessThan(1e-7);
    } finally {
      world.dispose();
    }
  } finally {
    loader.dispose();
    await rm(cache, { recursive: true, force: true });
  }
});
