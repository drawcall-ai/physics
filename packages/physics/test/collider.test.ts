import { expect, it } from "vitest";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  SphereCollider,
} from "../src/index.js";

it("captures immutable dimensions and independent material/group settings", () => {
  const size: [number, number, number] = [1, 2, 3];
  const material = { density: 12 };
  const groups = { membership: 1, filter: 2 };
  const box = new BoxCollider({ size })
    .setMaterial(material)
    .setCollisionGroups(groups)
    .setSensor(true);
  size[0] = 9;
  material.density = 99;
  groups.filter = 0;
  expect(box.size).toEqual([1, 2, 3]);
  expect(Object.isFrozen(box.size)).toBe(true);
  expect(Reflect.set(box, "size", [9, 9, 9])).toBe(false);
  const copy = box
    .clone()
    .setMaterial({ density: 20 })
    .setSensor(false)
    .setCollisionGroups(undefined);
  expect(copy.size).not.toBe(box.size);
  expect(box.material?.density).toBe(12);
  expect(box.sensor).toBe(true);
  expect(box.collisionGroups).toEqual({ membership: 1, filter: 2 });
  expect(() => box.setMaterial({ restitution: 2 })).toThrow("material");
  expect(() => box.copy(new BoxCollider())).toThrow("immutable");
  expect(box.sensor).toBe(true);
});

it("clones configured primitive dimensions and rejects mismatched copies", () => {
  for (const collider of [
    new SphereCollider({ radius: 2 }),
    new CapsuleCollider({ radius: 2, length: 3 }),
    new CylinderCollider({ radius: 2, height: 4 }),
  ]) {
    expect(collider.clone().shape()).toEqual(collider.shape());
    expect(Reflect.set(collider, "radius", 9)).toBe(false);
  }
  expect(() =>
    new SphereCollider().copy(new SphereCollider({ radius: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CapsuleCollider().copy(new CapsuleCollider({ length: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CylinderCollider().copy(new CylinderCollider({ height: 2 })),
  ).toThrow("immutable");
});

it("rejects invalid dimensions at construction", () => {
  expect(() => new BoxCollider({ size: [1, 0, 1] })).toThrow(
    "positive and finite",
  );
  expect(() => new SphereCollider({ radius: NaN })).toThrow(
    "positive and finite",
  );
  expect(() => new CapsuleCollider({ length: -1 })).toThrow(
    "positive and finite",
  );
  expect(() => new CylinderCollider({ height: Infinity })).toThrow(
    "positive and finite",
  );
  expect(() => Reflect.construct(BoxCollider, [{ size: [1, 2] }])).toThrow(
    "positive and finite",
  );
});
