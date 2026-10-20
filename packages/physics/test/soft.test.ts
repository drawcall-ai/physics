import { expect, it } from "vitest";
import {
  BoxGeometry,
  Group,
  Mesh,
  PlaneGeometry,
  SphereGeometry,
  Vector3,
} from "three";
import {
  clone,
  RigidBody,
  SoftAttachment,
  SoftBody,
  type SoftMaterial,
} from "../src/index.js";
import {
  selectSoftNodes,
  surfaceMesh,
  updateSoftGeometry,
} from "../src/backend.js";

const material: SoftMaterial = {
  density: 1000,
  elasticity: { model: "spring", stretch: { frequency: 20, dampingRatio: 1 } },
};

it("keeps rest geometry immutable and remaps cloned skins and attachments", () => {
  const rest = new PlaneGeometry(1, 1);
  const body = new SoftBody({
    shape: { kind: "surface", geometry: rest, thickness: 0.001 },
    material,
  });
  body.setLinearDamping(0.4);
  expect(() => body.setLinearDamping(-1)).toThrow("Damping");
  expect(() => body.setLinearDamping(NaN)).toThrow("Damping");
  body.add(new Mesh(body.geometry));
  const target = new RigidBody({ bodyType: "static" });
  const mount = new SoftAttachment({ body, region: [0], target });
  const root = new Group().add(body, mount, target);
  const original = body.geometry.getAttribute("position").getX(0);
  const positions = surfaceMesh(body.geometry).positions;
  const first = positions[0];
  if (first === undefined) throw new Error("Missing vertex");
  positions[0] = first + 0.2;
  updateSoftGeometry(body, positions);
  expect(rest.getAttribute("position").getX(0)).toBe(original);
  const copy = clone(root);
  const clonedBody = copy.children[0],
    attachment = copy.children[1];
  if (
    !(clonedBody instanceof SoftBody) ||
    !(attachment instanceof SoftAttachment)
  )
    throw new Error("Incorrect clones");
  const mesh = clonedBody.children[0];
  if (!(mesh instanceof Mesh)) throw new Error("Missing cloned skin");
  expect(clonedBody.linearDamping).toBe(0.4);
  expect(mesh.geometry).toBe(clonedBody.geometry);
  expect(mesh.geometry).not.toBe(body.geometry);
  expect(mesh.geometry.getAttribute("position").getX(0)).toBe(original);
  expect(attachment.options.body).toBe(clonedBody);
  expect(attachment.options.target).toBe(copy.children[2]);
});

it("welds render seams without discarding the render vertex mapping", () => {
  const box = new BoxGeometry();
  const mesh = surfaceMesh(box);
  expect(mesh.positions.length / 3).toBe(8);
  expect(mesh.vertices.length).toBe(box.getAttribute("position").count);
  const solid: SoftMaterial = {
    density: 1000,
    elasticity: { model: "neoHookean", youngModulus: 1000, poissonRatio: 0.3 },
  };
  expect(
    () =>
      new SoftBody({
        shape: {
          kind: "volume",
          geometry: new SphereGeometry(0.1),
          cellSize: 0.05,
        },
        material: solid,
      }),
  ).not.toThrow();
  expect(
    () =>
      new SoftBody({
        shape: { kind: "volume", geometry: new PlaneGeometry(), cellSize: 0.1 },
        material: solid,
      }),
  ).toThrow("closed");
});

it("validates attachment selection against rest nodes and rejects empty regions", () => {
  const body = new SoftBody({
    shape: { kind: "surface", geometry: new PlaneGeometry(), thickness: 0.01 },
    material,
  });
  const positions = surfaceMesh(body.geometry).positions;
  const top = new SoftAttachment({
    body,
    target: null,
    region: { min: [-1, 0, -1], max: [1, 1, 1] },
  });
  expect(selectSoftNodes(top, positions)).toEqual([0, 1]);
  const empty = new SoftAttachment({
    body,
    target: null,
    region: { min: [2, 2, 2], max: [3, 3, 3] },
  });
  expect(() => selectSoftNodes(empty, positions)).toThrow("selects no");
  expect(() => body.applyForce(new Vector3())).toThrow("built world");
});

it("rejects invalid volume topology before passing it to a native solver", () => {
  const positions = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
  const solid: SoftMaterial = {
    density: 1000,
    elasticity: { model: "neoHookean", youngModulus: 1000, poissonRatio: 0.3 },
  };
  expect(
    () =>
      new SoftBody({
        shape: {
          kind: "volume",
          positions,
          tetrahedra: [0, 1, 2, 3, 0, 1, 2, 3],
        },
        material: solid,
      }),
  ).toThrow("Duplicate tetrahedron");
  expect(
    () =>
      new SoftBody({
        shape: {
          kind: "volume",
          positions: [...positions, 2, 2, 2],
          tetrahedra: [0, 1, 2, 3],
        },
        material: solid,
      }),
  ).toThrow("unused");
  const inward = new BoxGeometry();
  inward.scale(-1, 1, 1);
  expect(
    () =>
      new SoftBody({
        shape: { kind: "volume", geometry: inward, cellSize: 0.2 },
        material: solid,
      }),
  ).toThrow("outward winding");
});

it("rejects elasticity properties that would otherwise be silently ignored", () => {
  const elasticity = {
    model: "saintVenantKirchhoff",
    youngModulus: 1000,
    poissonRatio: 0.3,
    dampingRatio: 1,
  } as const;
  expect(
    () =>
      new SoftBody({
        shape: {
          kind: "surface",
          geometry: new PlaneGeometry(),
          thickness: 0.01,
        },
        material: { density: 1000, elasticity },
      }),
  ).toThrow("dampingRatio");
});

it("requires a single elastic damping convention", () => {
  expect(
    () =>
      new SoftBody({
        shape: { kind: "volume", geometry: new BoxGeometry(), cellSize: 0.2 },
        material: {
          density: 1000,
          elasticity: {
            model: "neoHookean",
            youngModulus: 1000,
            poissonRatio: 0.3,
            dampingRatio: 0.1,
            dampingTime: 0.002,
          },
        },
      }),
  ).toThrow("either dampingRatio or dampingTime");
});
