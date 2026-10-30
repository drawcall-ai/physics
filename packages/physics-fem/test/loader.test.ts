import { writeFile } from "node:fs/promises";
import { join } from "node:path";
import { pathToFileURL } from "node:url";
import { expect, it, vi } from "vitest";
import {
  BoxGeometry,
  Float32BufferAttribute,
  Loader,
  LoadingManager,
  Mesh,
  MeshBasicMaterial,
  type Object3D,
} from "three";
import { SoftBody } from "@drawcall/physics";
import { updateReducedGeometry } from "@drawcall/physics/backend";
import { FEMLoader } from "../src/loader.js";
import { fixture, folder, options, reducer, sourceFolder } from "./fixtures.js";

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
    dispose = vi.fn();
    override loadAsync(url: string): Promise<Object3D> {
      return loaded(url);
    }
  }
  const handler = new SourceLoader(manager);
  manager.addHandler(/\.glts$/, handler);
  reducer.mockImplementation(async (input) => fixture(input));
  const loader = new FEMLoader(manager)
    .setPath("/assets/")
    .setCachePath(await folder());
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
  loader.dispose();
  expect(handler.dispose).not.toHaveBeenCalled();
});

it("loads GLTS directly and releases owned scripts on failure and disposal", async () => {
  const directory = await sourceFolder();
  const url = pathToFileURL(join(directory, "finger.glts")).href;
  const disposed = vi.fn();
  vi.stubGlobal("femDisposed", disposed);
  await writeFile(
    pathToFileURL(join(directory, "finger.glts")),
    `
    import { BoxGeometry, Mesh } from "three";
    import { scene, onDispose } from "@drawcall/glts";
    scene.name = "procedural finger";
    scene.add(new Mesh(new BoxGeometry()));
    onDispose(() => globalThis.femDisposed());
  `,
  );
  const loader = new FEMLoader().setCachePath(await folder());
  try {
    reducer.mockRejectedValueOnce(new Error("preparation failed"));
    await expect(loader.loadAsync({ url, ...options })).rejects.toThrow(
      "preparation failed",
    );
    expect(disposed).toHaveBeenCalledTimes(1);
    reducer.mockImplementation(async (input) => fixture(input));
    const body = await loader.loadAsync({ url, ...options });
    expect(body).toBeInstanceOf(SoftBody);
    expect(body.name).toBe("procedural finger");
    expect(body.children[0]?.children[0]).toBeInstanceOf(Mesh);
    expect(body.model?.renderPositions).toHaveLength(24);
    reducer.mockImplementationOnce(async (input) => {
      loader.dispose();
      return fixture(input);
    });
    await expect(
      loader.loadAsync({
        url,
        ...options,
        material: { ...options.material, density: 1200 },
      }),
    ).rejects.toThrow("disposed");
    expect(disposed).toHaveBeenCalledTimes(3);
    await expect(loader.loadAsync({ url, ...options })).rejects.toThrow(
      "disposed",
    );
  } finally {
    loader.dispose();
  }
});

it("caches physical inputs while allowing visual and contact-material changes without rebaking", async () => {
  const object = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  reducer.mockImplementation(async (input) => fixture(input));
  const loader = new FEMLoader().setCachePath(await folder());
  await loader.loadAsync({ object, ...options });
  object.name = "new name";
  object.position.x = 5;
  object.material.color.set("red");
  const body = await loader.loadAsync({
    object,
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
  await loader.loadAsync({ object, ...options });
  expect(reducer).toHaveBeenCalledTimes(2);
  await loader.loadAsync({
    object,
    ...options,
    material: { ...options.material, density: 1500 },
  });
  expect(reducer).toHaveBeenCalledTimes(3);
  await loader.loadAsync({
    object,
    ...options,
    mount: { min: [-1, -1, -1], max: [1, 0.1, 1] },
  });
  expect(reducer).toHaveBeenCalledTimes(4);
});

it("rejects invalid preparation results without persisting them", async () => {
  const object = new Mesh(new BoxGeometry());
  const loader = new FEMLoader().setCachePath(await folder());
  reducer.mockImplementation(async () =>
    new TextEncoder().encode('{"version":99}'),
  );
  await expect(loader.loadAsync({ object, ...options })).rejects.toThrow(
    "version",
  );
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, mount: { min: [-1, -1, -1], max: [1, 0.1, 1] } }),
  );
  await expect(loader.loadAsync({ object, ...options })).rejects.toThrow(
    "physical recipe",
  );
  reducer.mockImplementation(async (input) =>
    fixture({ ...input, material: { ...input.material, density: 800 } }),
  );
  await expect(loader.loadAsync({ object, ...options })).rejects.toThrow(
    "baked",
  );
  reducer.mockReset();
  reducer.mockImplementation(async (input) => fixture(input));
  await expect(
    loader.loadAsync({ object, ...options }),
  ).resolves.toBeInstanceOf(SoftBody);
  expect(reducer).toHaveBeenCalledTimes(1);
});

it("rejects edits to baked visual topology, hierarchy and transforms", async () => {
  reducer.mockImplementation(async (input) => fixture(input));
  const body = await new FEMLoader()
    .setCachePath(await folder())
    .loadAsync({ object: new Mesh(new BoxGeometry()), ...options });
  const changes: ((mesh: Mesh) => void)[] = [
    (mesh) => {
      mesh.geometry.getAttribute("position").needsUpdate = true;
    },
    (mesh) => {
      mesh.geometry.setAttribute("position", new Float32BufferAttribute(72, 3));
    },
    (mesh) => {
      if (mesh.geometry.index) mesh.geometry.index.needsUpdate = true;
    },
    (mesh) => {
      mesh.geometry = mesh.geometry.clone();
    },
    (mesh) => {
      mesh.position.x = 0.1;
    },
    (mesh) => {
      mesh.removeFromParent();
    },
    (mesh) => {
      mesh.add(new Mesh());
    },
  ];
  for (const change of changes) {
    const clone = body.clone(),
      mesh = clone.children[0];
    if (!(mesh instanceof Mesh) || !clone.model)
      throw new Error("Missing cloned visual");
    const positions = clone.model.renderPositions;
    updateReducedGeometry(clone, positions);
    change(mesh);
    expect(() => updateReducedGeometry(clone, positions)).toThrow(
      /reload the component/,
    );
  }
});

it.each([
  { coordinates: [0] },
  { coordinates: [7, 7] },
  { triangles: [0, 1, 1000] },
  { basis: Array(72).fill(0) },
  { mass: [1] },
  { stiffness: [1, 1, 0, 0, 1, 0, 0, 0, 1] },
  { gravity: Array(18).fill(NaN) },
  { quadrature: { weights: [-1], gradients: Array(27).fill(0) } },
  { surfaceError: { contact: -1, render: 0 } },
  { mountRegion: { min: [1, 0, 0], max: [0, 1, 1] } },
])(
  "rejects invalid reduced mechanics before constructing a body: %j",
  async (changes) => {
    reducer.mockImplementation(async (input) => fixture(input, changes));
    const loader = new FEMLoader().setCachePath(await folder());
    await expect(
      loader.loadAsync({ object: new Mesh(new BoxGeometry()), ...options }),
    ).rejects.toThrow();
  },
);
