import { expect, it } from "vitest";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  SphereCollider,
} from "../src/index.js";

it("keeps the existing primitive defaults", () => {
  expect(new BoxCollider().shape()).toEqual({ kind: "box", size: [1, 1, 1] });
  expect(new SphereCollider().shape()).toEqual({ kind: "sphere", radius: 0.5 });
  expect(new CapsuleCollider().shape()).toEqual({
    kind: "capsule",
    radius: 0.5,
    length: 1,
  });
  expect(new CylinderCollider().shape()).toEqual({
    kind: "cylinder",
    radius: 0.5,
    height: 1,
  });
});

it("captures primitive dimensions without retaining mutable constructor inputs", () => {
  const size: [number, number, number] = [1, 2, 3];
  const box = new BoxCollider({ size });
  const options = { radius: 2, length: 3, height: 4 };
  const sphere = new SphereCollider(options);
  const capsule = new CapsuleCollider(options);
  const cylinder = new CylinderCollider(options);
  size[0] = 9;
  options.radius = 9;
  options.length = 9;
  options.height = 9;
  expect(box.size).toEqual([1, 2, 3]);
  expect(box.size).not.toBe(size);
  expect(Object.isFrozen(box.size)).toBe(true);
  expect(sphere.radius).toBe(2);
  expect(capsule.shape()).toEqual({ kind: "capsule", radius: 2, length: 3 });
  expect(cylinder.shape()).toEqual({ kind: "cylinder", radius: 2, height: 4 });
  expect(Reflect.set(box, "size", [9, 9, 9])).toBe(false);
  expect(Reflect.set(sphere, "radius", 9)).toBe(false);
  expect(Reflect.set(capsule, "radius", 9)).toBe(false);
  expect(Reflect.set(capsule, "length", 9)).toBe(false);
  expect(Reflect.set(cylinder, "radius", 9)).toBe(false);
  expect(Reflect.set(cylinder, "height", 9)).toBe(false);
});

it("clones configured shapes and independently copies mutable collider settings", () => {
  const colliders = [
    new BoxCollider({ size: [1, 2, 3] }),
    new SphereCollider({ radius: 2 }),
    new CapsuleCollider({ radius: 2, length: 3 }),
    new CylinderCollider({ radius: 2, height: 4 }),
  ];
  for (const collider of colliders) {
    collider
      .setMaterial({ density: 12 })
      .setSensor(true)
      .setCollisionGroups({ membership: 1, filter: 2 });
    const copy = collider.clone();
    expect(Object.getPrototypeOf(copy)).toBe(Object.getPrototypeOf(collider));
    expect(copy.shape()).toEqual(collider.shape());
    expect(copy.material).toEqual(collider.material);
    expect(copy.material).not.toBe(collider.material);
    copy
      .setMaterial({ density: 20 })
      .setSensor(false)
      .setCollisionGroups(undefined);
    expect(collider.material?.density).toBe(12);
    expect(collider.sensor).toBe(true);
    expect(collider.collisionGroups).toEqual({ membership: 1, filter: 2 });
  }
  const box = new BoxCollider({ size: [1, 2, 3] });
  expect(box.clone().size).not.toBe(box.size);
});

it("rejects copying different immutable dimensions before changing the destination", () => {
  const box = new BoxCollider().setSensor(true);
  expect(() => box.copy(new BoxCollider({ size: [2, 1, 1] }))).toThrow(
    "immutable",
  );
  expect(box.sensor).toBe(true);
  expect(box.size).toEqual([1, 1, 1]);
  expect(() =>
    new SphereCollider().copy(new SphereCollider({ radius: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CapsuleCollider().copy(new CapsuleCollider({ radius: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CapsuleCollider().copy(new CapsuleCollider({ length: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CylinderCollider().copy(new CylinderCollider({ radius: 2 })),
  ).toThrow("immutable");
  expect(() =>
    new CylinderCollider().copy(new CylinderCollider({ height: 2 })),
  ).toThrow("immutable");
  box.copy(new BoxCollider().setMaterial({ density: 5 }));
  expect(box.material).toEqual({ density: 5 });
  expect(box.sensor).toBe(false);
});

it.each([0, -1, NaN, Infinity])(
  "rejects invalid primitive dimensions %s during construction",
  (value) => {
    expect(() => new BoxCollider({ size: [1, value, 1] })).toThrow(
      "positive and finite",
    );
    expect(() => new SphereCollider({ radius: value })).toThrow(
      "positive and finite",
    );
    expect(() => new CapsuleCollider({ radius: value })).toThrow(
      "positive and finite",
    );
    expect(() => new CapsuleCollider({ length: value })).toThrow(
      "positive and finite",
    );
    expect(() => new CylinderCollider({ radius: value })).toThrow(
      "positive and finite",
    );
    expect(() => new CylinderCollider({ height: value })).toThrow(
      "positive and finite",
    );
  },
);

it("rejects an incomplete box size from JavaScript callers", () => {
  expect(() => Reflect.construct(BoxCollider, [{ size: [1, 2] }])).toThrow(
    "positive and finite",
  );
});
