import { expect, test } from "vitest";
import { Scene } from "three";
import { SoftBody } from "@drawcall/physics";
import { buildWorld as rapierWorld } from "@drawcall/physics-rapier";
import { buildWorld as mujocoWorld } from "@drawcall/physics-mujoco";
import {
  cargo,
  hammock,
  simulationOptions,
  fabrics,
  type Backend,
} from "../model";

const cases: { backend: Backend; settle: number }[] = [
  { backend: "rapier", settle: 8 },
  { backend: "mujoco", settle: 3 },
];
const lowest = (positions: Float32Array) =>
  Math.min(...Array.from(positions).filter((_, i) => i % 3 === 1));

test.each(cases)(
  "$backend cloth catches cargo after $settle seconds, switches fabric and resets",
  async ({ backend, settle }) => {
    const model = hammock(backend);
    const scene = new Scene().add(model.root);
    const buildWorld = backend === "rapier" ? rapierWorld : mujocoWorld;
    const options = simulationOptions(backend);
    const world = await buildWorld({ scene, ...options });
    const advance = (seconds: number) => {
      for (let i = 0; i < Math.round(seconds * 60); i++) world.update(1 / 60);
    };
    try {
      const rest = model.cloth.getPositions();
      world.reset();
      advance(settle);
      const empty = model.cloth.getPositions();
      expect(Array.from(empty).every(Number.isFinite)).toBe(true);
      expect(lowest(empty)).toBeLessThan(1.9);
      expect(lowest(empty)).toBeGreaterThan(0.3);

      const load = cargo(backend, "foam", 0);
      if (!(load instanceof SoftBody))
        throw new Error("Foam cargo must deform");
      scene.add(load);
      let caught = false;
      const stopSampling = world.onAfterStep(() => {
        caught ||= model.cloth
          .getContacts()
          .some((contact) => contact.otherBody === load);
      });
      advance(0.75);
      expect(lowest(load.getPositions())).toBeGreaterThan(0.2);
      expect(lowest(load.getPositions())).toBeLessThan(2);
      advance(2.25);
      stopSampling();
      const loaded = model.cloth.getPositions();
      if (backend === "mujoco") expect(caught).toBe(true);
      expect(Array.from(loaded).every(Number.isFinite)).toBe(true);
      expect(lowest(loaded)).toBeGreaterThan(0.2);
      // Cargo can roll off the open sides after being caught.
      expect(lowest(load.getPositions())).toBeGreaterThan(-0.1);

      const fabric = fabrics(backend).stretch;
      if (!fabric) throw new Error("Missing stretch fabric");
      const beforeMaterial = model.cloth.getPositions();
      model.cloth.setMaterial(fabric);
      world.update(0);
      expect(Array.from(model.cloth.getPositions())).toEqual(
        Array.from(beforeMaterial),
      );
      advance(2);
      expect(
        Array.from(model.cloth.getPositions()).every(Number.isFinite),
      ).toBe(true);
      expect(load.getPositions().every(Number.isFinite)).toBe(true);

      load.removeFromParent();
      world.reset();
      const reset = model.cloth.getPositions();
      expect(reset.length).toBe(rest.length);
      for (const [i, value] of rest.entries())
        expect(reset[i]).toBeCloseTo(value, 4);

      for (const kind of ["rubber", "rigid"] as const) {
        const body = cargo(backend, kind, 0);
        scene.add(body);
        advance(2);
        if (body instanceof SoftBody)
          expect(body.getPositions().every(Number.isFinite)).toBe(true);
        else expect(body.position.toArray().every(Number.isFinite)).toBe(true);
        body.removeFromParent();
        world.reset();
      }
    } finally {
      world.dispose();
    }
  },
);
