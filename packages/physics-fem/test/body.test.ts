import { expect, it } from "vitest";
import { BoxGeometry, Float32BufferAttribute, Mesh } from "three";
import { updateReducedGeometry } from "@drawcall/physics/backend";
import { prepareSoftBody } from "../src/body.js";
import { fixture, options, reducer } from "./fixtures.js";

it("rejects edits to baked visual topology, hierarchy and transforms", async () => {
  reducer.mockImplementation(async (input) => fixture(input));
  const body = await prepareSoftBody(new Mesh(new BoxGeometry()), options);
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
    await expect(
      prepareSoftBody(new Mesh(new BoxGeometry()), options),
    ).rejects.toThrow();
  },
);
