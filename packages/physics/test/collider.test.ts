import { expect, expectTypeOf, it } from "vitest";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  SphereCollider,
} from "../src/index.js";

it("keeps immutable dimensions and independent material/group settings", () => {
  const material = { density: 12 };
  const groups = { membership: 1, filter: 2 };
  const box = new BoxCollider({ size: [1, 2, 3] })
    .setMaterial(material)
    .setCollisionGroups(groups);
  material.density = 99;
  groups.filter = 0;
  expect(box.size).toEqual([1, 2, 3]);
  expectTypeOf<Pick<BoxCollider, "size">>().toEqualTypeOf<{
    readonly size: readonly [number, number, number];
  }>();
  const copy = box
    .clone()
    .setMaterial({ density: 20 })
    .setCollisionGroups(undefined);
  expect(copy.size).toEqual(box.size);
  expect(box.material?.density).toBe(12);
  expect(box.collisionGroups).toEqual({ membership: 1, filter: 2 });
  expect(() => box.setMaterial({ restitution: 2 })).toThrow("material");
});

it("clones configured primitive dimensions", () => {
  for (const collider of [
    new SphereCollider({ radius: 2 }),
    new CapsuleCollider({ radius: 2, height: 3 }),
    new CylinderCollider({ radius: 2, height: 4 }),
  ]) {
    expect(collider.clone().shape()).toEqual(collider.shape());
    expectTypeOf<Pick<typeof collider, "radius">>().toEqualTypeOf<{
      readonly radius: number;
    }>();
  }
});

it("rejects invalid dimensions at construction", () => {
  expect(() => new BoxCollider({ size: [1, 0, 1] })).toThrow(
    "positive and finite",
  );
  expect(() => new SphereCollider({ radius: NaN })).toThrow(
    "positive and finite",
  );
  expect(() => new CapsuleCollider({ height: -1 })).toThrow(
    "positive and finite",
  );
  expect(() => new CylinderCollider({ height: Infinity })).toThrow(
    "positive and finite",
  );
});
