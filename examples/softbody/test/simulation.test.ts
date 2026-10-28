import { expect, test } from "vitest";
import { buildWorld } from "@drawcall/physics-mujoco";
import { SoftBody } from "@drawcall/physics";
import { gripper, simulationOptions } from "../model.ts";
import type { PayloadKind } from "../payload.ts";

test.each<PayloadKind>(["block", "cloth", "rope"])(
  "deformable pads lift, hold, release, and reset %s",
  async (kind) => {
    const model = await gripper(
      new URL("../public/pad.glts", import.meta.url).href,
      kind,
    );
    const world = await buildWorld({
      scene: model.scene,
      ...simulationOptions,
    });
    const target = { grip: 0, lift: 0 };
    world.onBeforeStep(() => model.update(target, world.fixedDelta));
    const step = (seconds: number) => {
      for (let i = 0; i < seconds / world.fixedDelta; i++)
        world.update(world.fixedDelta);
    };
    const positions = () =>
      model.object instanceof SoftBody
        ? model.object.getPositions()
        : model.object.position.toArray();
    const rest = positions();
    const restHeight = model.height();
    const pads = model.pads.map((pad) => pad.getPositions());
    try {
      step(0.5);
      target.grip = 1;
      step(2);
      for (const pad of model.pads)
        expect(
          pad
            .getContacts()
            .some(
              (contact) =>
                contact.otherBody === model.object && contact.force > 0,
            ),
        ).toBe(true);
      target.lift = 1;
      step(3);
      const held = model.height();
      if (model.object instanceof SoftBody)
        expect(
          Math.min(
            ...model.object.getPositions().filter((_, i) => i % 3 === 1),
          ),
        ).toBeGreaterThan(0.05);
      else expect(held - restHeight).toBeGreaterThan(0.15);
      step(3);
      expect(Math.abs(model.height() - held)).toBeLessThan(
        kind === "block" ? 0.001 : 0.005,
      );
      target.grip = 0;
      step(2);
      expect(model.height()).toBeLessThan(0.06);
      expect(
        model.object
          .getContacts()
          .some(
            (contact) =>
              model.pads.some((pad) => pad === contact.otherBody) &&
              contact.force > 0,
          ),
      ).toBe(false);

      model.reset();
      world.reset();
      target.grip = target.lift = 0;
      expect(positions()).toEqual(rest);
      model.pads.forEach((pad, i) =>
        expect(pad.getPositions()).toEqual(pads[i]),
      );

      // A cord can catch on the deformed pad corners even without friction.
      if (kind === "rope") return;
      for (const body of [model.object, ...model.pads]) {
        if (body instanceof SoftBody)
          body.setMaterial({
            ...body.material,
            staticFriction: 0.001,
            dynamicFriction: 0.001,
          });
        else
          body.setMaterial({ staticFriction: 0.001, dynamicFriction: 0.001 });
      }
      target.grip = 1;
      step(2);
      target.lift = 1;
      step(3);
      expect(model.height()).toBeLessThan(0.06);
    } finally {
      world.dispose();
      model.dispose();
    }
  },
);
