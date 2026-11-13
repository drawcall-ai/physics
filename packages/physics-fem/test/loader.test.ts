import { mkdir, writeFile } from "node:fs/promises";
import { expect, it, vi } from "vitest";
import {
  BoxGeometry,
  Loader,
  LoadingManager,
  Mesh,
  MeshBasicMaterial,
  type Object3D,
} from "three";
import { SoftBody } from "@drawcall/physics";
import { updateReducedGeometry } from "@drawcall/physics/backend";
import { CachedFEMLoader } from "../src/loader.js";
import { fixture, options, reducer } from "./fixtures.js";

const url = "part.glts";

/** A loader whose handler returns a fresh copy of `object` in its current state. */
function loaderOf(object: Object3D): CachedFEMLoader {
  const manager = new LoadingManager();
  class SourceLoader extends Loader<Object3D> {
    override async loadAsync(): Promise<Object3D> {
      return object.clone();
    }
  }
  manager.addHandler(/\.glts$/, new SourceLoader(manager));
  return new CachedFEMLoader(manager);
}

it("uses the registered asset handler and returns independent SoftBodies with their visuals", async () => {
  const object = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  object.name = "finger";
  object.scale.setScalar(2);
  const manager = new LoadingManager();
  const roots: Object3D[] = [];
  const loaded = vi.fn<(url: string) => Promise<Object3D>>(async () => {
    const root = object.clone();
    roots.push(root);
    return root;
  });
  class SourceLoader extends Loader<Object3D> {
    override loadAsync(url: string): Promise<Object3D> {
      return loaded(url);
    }
  }
  const handler = new SourceLoader(manager);
  manager.addHandler(/\.glts$/, handler);
  reducer.mockImplementation(async (input) => fixture(input));
  const loader = new CachedFEMLoader(manager).setPath("/assets/");
  const first = await loader.loadAsync({ url: "finger.glts?v=2", ...options });
  const second = await loader.loadAsync({ url: "finger.glts", ...options });
  expect(loaded).toHaveBeenCalledWith("/assets/finger.glts?v=2");
  expect(first).toBeInstanceOf(SoftBody);
  expect(first).not.toBe(second);
  expect(first.geometry).not.toBe(second.geometry);
  expect(first.children[0]).toBe(roots[0]);
  expect(second.children[0]).toBe(roots[1]);
  expect(first.name).toBe("finger");
  expect(object.parent).toBe(null);
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

it("loads root-absolute GLTS from publicDir in Node and releases the script when preparation fails", async () => {
  const disposed = vi.fn();
  vi.stubGlobal("femDisposed", disposed);
  await mkdir("public/parts", { recursive: true });
  await writeFile(
    "public/parts/finger.glts",
    `
    import { BoxGeometry, Mesh } from "three";
    import { scene, onDispose } from "@drawcall/glts";
    scene.name = "procedural finger";
    scene.add(new Mesh(new BoxGeometry()));
    onDispose(() => globalThis.femDisposed());
  `,
  );
  const request = { url: "finger.glts", ...options };
  const loader = new CachedFEMLoader().setPath("/parts/");
  reducer.mockRejectedValueOnce(new Error("preparation failed"));
  await expect(loader.loadAsync(request)).rejects.toThrow("preparation failed");
  expect(disposed).toHaveBeenCalledTimes(1);
  reducer.mockImplementation(async (input) => fixture(input));
  const body = await loader.loadAsync(request);
  expect(body.name).toBe("procedural finger");
  expect(body.model?.renderPositions).toHaveLength(24);
  expect(disposed).toHaveBeenCalledTimes(1);
  const scene = body.children[0];
  if (!scene || !("dispose" in scene) || typeof scene.dispose !== "function")
    throw new Error("Expected the GLTS scene as the body's visual");
  scene.dispose();
  expect(disposed).toHaveBeenCalledTimes(2);
});

it("caches physical inputs while allowing visual and contact-material changes without rebaking", async () => {
  const object = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  const loader = loaderOf(object);
  reducer.mockImplementation(async (input) => fixture(input));
  await loader.loadAsync({ url, ...options });
  object.name = "new name";
  object.position.x = 5;
  object.material.color.set("red");
  const body = await loader.loadAsync({
    url,
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
  await loader.loadAsync({ url, ...options });
  expect(reducer).toHaveBeenCalledTimes(2);
  await loader.loadAsync({
    url,
    ...options,
    material: { ...options.material, density: 1500 },
  });
  expect(reducer).toHaveBeenCalledTimes(3);
  await loader.loadAsync({
    url,
    ...options,
    mount: { min: [-1, -1, -1], max: [1, 0.1, 1] },
  });
  expect(reducer).toHaveBeenCalledTimes(4);
});

it("rejects invalid preparation results without persisting them", async () => {
  const loader = loaderOf(new Mesh(new BoxGeometry()));
  reducer.mockImplementation(async () =>
    new TextEncoder().encode('{"version":99}'),
  );
  await expect(loader.loadAsync({ url, ...options })).rejects.toThrow(
    "version",
  );
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, mount: { min: [-1, -1, -1], max: [1, 0.1, 1] } }),
  );
  await expect(loader.loadAsync({ url, ...options })).rejects.toThrow(
    "physical recipe",
  );
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, material: { ...input.material, density: 800 } }),
  );
  await expect(loader.loadAsync({ url, ...options })).rejects.toThrow("baked");
  reducer.mockReset();
  reducer.mockImplementation(async (input) => fixture(input));
  await expect(loader.loadAsync({ url, ...options })).resolves.toBeInstanceOf(
    SoftBody,
  );
  expect(reducer).toHaveBeenCalledTimes(1);
});
