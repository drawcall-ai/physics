import { BoxGeometry, Group, Mesh, MeshBasicMaterial, Vector3 } from "three";
import { expect, it } from "vitest";
import { source } from "../src/source.js";

it("snapshots physical geometry while preserving visual seams and caller ownership", () => {
  const geometry = new BoxGeometry(0.01, 0.02, 0.03);
  const material = new MeshBasicMaterial();
  const mesh = new Mesh(geometry, material);
  mesh.position.x = 0.01;
  const group = new Group().add(mesh);
  group.position.y = 2;
  group.scale.setScalar(2);
  const result = source(group);
  expect(result.positions).toHaveLength(8 * 3);
  expect(result.skins).toHaveLength(1);
  const skin = result.skins[0];
  if (!skin) throw new Error("Missing skin");
  expect(skin.vertices).toHaveLength(24);
  expect(skin.mesh.geometry).not.toBe(geometry);
  expect(skin.mesh.material).toBe(material);
  expect(mesh.parent).toBe(group);
  expect(mesh.geometry).toBe(geometry);
  expect(new Vector3().setFromMatrixPosition(result.pose).y).toBe(2);
  expect(
    Math.min(...result.positions.filter((_, i) => i % 3 === 0)),
  ).toBeCloseTo(0.01);
});

it("keeps scene identity and visual changes out of the physical snapshot", () => {
  const mesh = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
  const before = source(mesh);
  mesh.position.set(10, 20, 30);
  mesh.name = "renamed";
  mesh.material.color.set("red");
  const after = source(mesh);
  expect(after.positions).toEqual(before.positions);
  expect(after.triangles).toEqual(before.triangles);
  mesh.geometry.translate(0.1, 0, 0);
  expect(source(mesh).positions).not.toEqual(before.positions);
});

it("rejects partial geometry and reflected transforms instead of baking a different solid", () => {
  const mesh = new Mesh(new BoxGeometry());
  mesh.geometry.setDrawRange(0, 3);
  expect(() => source(mesh)).toThrow("partial");
  mesh.geometry.setDrawRange(0, Infinity);
  mesh.scale.x = -1;
  expect(() => source(mesh)).toThrow("positive");
});
