import { expect, it } from "vitest";
import {
  BufferGeometry,
  InterleavedBuffer,
  InterleavedBufferAttribute,
} from "three";
import { matchesGeometry, snapshotGeometry } from "../src/geometry.js";

it("compares interpreted positions while ignoring unrelated interleaved channels", () => {
  const data = new InterleavedBuffer(
    new Float32Array([1, 2, 3, 9, 4, 5, 6, 8, 7, 8, 9, 7]),
    4,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  const snapshot = snapshotGeometry(geometry);
  expect(snapshot.positions).toEqual([1, 2, 3, 4, 5, 6, 7, 8, 9]);
  data.array[3] = 99;
  expect(matchesGeometry(snapshot, geometry)).toBe(true);
  geometry.setAttribute("position", new InterleavedBufferAttribute(data, 3, 1));
  expect(matchesGeometry(snapshot, geometry)).toBe(false);
});

it("equates implicit and explicit sequential indices but detects changed topology and vertices", () => {
  const data = new InterleavedBuffer(
    new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    3,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  const snapshot = snapshotGeometry(geometry);
  geometry.setIndex([0, 1, 2]);
  expect(matchesGeometry(snapshot, geometry)).toBe(true);
  geometry.setIndex([0, 2, 1]);
  expect(matchesGeometry(snapshot, geometry)).toBe(false);
  geometry.setIndex(null);
  data.array[0] = 2;
  expect(matchesGeometry(snapshot, geometry)).toBe(false);
  expect(snapshot.positions[0]).toBe(0);
});
