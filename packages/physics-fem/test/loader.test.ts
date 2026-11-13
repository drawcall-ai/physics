import { expect, it } from "vitest";
import { BoxGeometry, Group, Mesh, MeshBasicMaterial } from "three";
import { SoftBody } from "@drawcall/physics";
import { updateReducedGeometry } from "@drawcall/physics/backend";
import { CachedFEMLoader } from "../src/loader.js";
import { fixture, options, reducer } from "./fixtures.js";

it("returns independent SoftBodies whose visuals are the created objects", async () => {
  const object = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  object.name = "finger";
  object.scale.setScalar(2);
  const roots: Mesh[] = [];
  const create = () => {
    const root = object.clone();
    roots.push(root);
    return root;
  };
  reducer.mockImplementation(async (input) => fixture(input));
  const loader = new CachedFEMLoader();
  const first = await loader.loadAsync({ object: create, ...options });
  const second = await loader.loadAsync({ object: create, ...options });
  expect(first).toBeInstanceOf(SoftBody);
  expect(first).not.toBe(second);
  expect(first.geometry).not.toBe(second.geometry);
  expect(first.children[0]).toBe(roots[0]);
  expect(second.children[0]).toBe(roots[1]);
  expect(first.name).toBe("finger");
  expect(reducer).toHaveBeenCalledTimes(1);
  const visual = first.children[0],
    other = second.children[0];
  if (!(visual instanceof Mesh) || !(other instanceof Mesh) || !first.model)
    throw new Error("Missing reduced visual geometry");
  const original = object.geometry.getAttribute("position").getY(0);
  updateReducedGeometry(
    first,
    first.model.renderPositions.map(
      (value, i) => value + (i % 3 === 1 ? 0.2 : 0),
    ),
  );
  expect(visual.geometry.getAttribute("position").getY(0)).toBeCloseTo(
    original + 0.1,
  );
  expect(other.geometry.getAttribute("position").getY(0)).toBe(original);
  expect(object.geometry.getAttribute("position").getY(0)).toBe(original);
  const clone = first.clone();
  const clonedVisual = clone.children[0];
  if (!(clonedVisual instanceof Mesh)) throw new Error("Missing cloned visual");
  expect(clone.model).toBe(first.model);
  expect(Object.isFrozen(first.model)).toBe(true);
  expect(Object.isFrozen(first.model.basis)).toBe(true);
  expect(Object.isFrozen(first.model.material.elasticity)).toBe(true);
  expect(clonedVisual.geometry).not.toBe(visual.geometry);
  expect(clonedVisual.geometry.getAttribute("position").getY(0)).toBe(original);
  updateReducedGeometry(
    clone,
    first.model.renderPositions.map((v, i) => v + (i % 3 === 1 ? 0.4 : 0)),
  );
  expect(clonedVisual.geometry.getAttribute("position").getY(0)).toBeCloseTo(
    original + 0.2,
  );
  expect(visual.geometry.getAttribute("position").getY(0)).toBeCloseTo(
    original + 0.1,
  );
  expect(() => clone.setMaterial({ ...clone.material, density: 2 })).toThrow(
    /baked/,
  );
  clone.scale.setScalar(2);
  expect(() => clone.validate()).toThrow(/scale is baked/);
});

it("rejects an object already attached to a scene", async () => {
  const object = new Mesh(new BoxGeometry());
  new Group().add(object);
  await expect(
    new CachedFEMLoader().loadAsync({ object: () => object, ...options }),
  ).rejects.toThrow("not attached");
});

it("caches physical inputs while allowing visual and contact-material changes without rebaking", async () => {
  const object = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  const loader = new CachedFEMLoader();
  const create = () => object.clone();
  reducer.mockImplementation(async (input) => fixture(input));
  await loader.loadAsync({ object: create, ...options });
  object.name = "new name";
  object.position.x = 5;
  object.material.color.set("red");
  const body = await loader.loadAsync({
    object: create,
    ...options,
    material: {
      ...options.material,
      staticFriction: 0.7,
      dynamicFriction: 0.7,
    },
  });
  expect(reducer).toHaveBeenCalledTimes(1);
  expect(body.material.staticFriction).toBe(0.7);
  expect(body.position.x).toBe(5);
  object.geometry.scale(2, 1, 1);
  await loader.loadAsync({ object: create, ...options });
  expect(reducer).toHaveBeenCalledTimes(2);
  await loader.loadAsync({
    object: create,
    ...options,
    material: { ...options.material, density: 1500 },
  });
  expect(reducer).toHaveBeenCalledTimes(3);
  await loader.loadAsync({
    object: create,
    ...options,
    mount: { min: [-1, -1, -1], max: [1, 0.1, 1] },
  });
  expect(reducer).toHaveBeenCalledTimes(4);
});

it("rejects invalid preparation results without persisting them", async () => {
  const loader = new CachedFEMLoader();
  const create = () => new Mesh(new BoxGeometry());
  reducer.mockImplementation(async () =>
    new TextEncoder().encode('{"version":99}'),
  );
  await expect(
    loader.loadAsync({ object: create, ...options }),
  ).rejects.toThrow("version");
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, mount: { min: [-1, -1, -1], max: [1, 0.1, 1] } }),
  );
  await expect(
    loader.loadAsync({ object: create, ...options }),
  ).rejects.toThrow("physical recipe");
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, material: { ...input.material, density: 800 } }),
  );
  await expect(
    loader.loadAsync({ object: create, ...options }),
  ).rejects.toThrow("baked");
  reducer.mockReset();
  reducer.mockImplementation(async (input) => fixture(input));
  await expect(
    loader.loadAsync({ object: create, ...options }),
  ).resolves.toBeInstanceOf(SoftBody);
  expect(reducer).toHaveBeenCalledTimes(1);
});
