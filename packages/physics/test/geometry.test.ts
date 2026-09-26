import { expect, it } from "vitest";
import {
  BufferGeometry,
  Float32BufferAttribute,
  InterleavedBuffer,
  InterleavedBufferAttribute,
} from "three";
import { matchesGeometry, snapshotGeometry } from "../src/geometry.js";

it("snapshots interpreted positions, ignoring unrelated interleaved channels", () => {
  const data = new InterleavedBuffer(
    new Float32Array([1, 2, 3, 9, 4, 5, 6, 8, 7, 8, 9, 7]),
    4,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  expect(snapshotGeometry(geometry).positions).toEqual([
    1, 2, 3, 4, 5, 6, 7, 8, 9,
  ]);
});

it("sees edits marked with needsUpdate and replaced attributes, as three.js requires", () => {
  const position = new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0], 3);
  const geometry = new BufferGeometry().setAttribute("position", position);
  const snapshot = snapshotGeometry(geometry);
  expect(matchesGeometry(snapshot, geometry)).toBe(true);
  position.setX(0, 2);
  position.needsUpdate = true;
  expect(matchesGeometry(snapshot, geometry)).toBe(false);
  const edited = snapshotGeometry(geometry);
  geometry.setIndex([0, 2, 1]);
  expect(matchesGeometry(edited, geometry)).toBe(false);
  const indexed = snapshotGeometry(geometry);
  geometry.setAttribute("position", position.clone());
  expect(matchesGeometry(indexed, geometry)).toBe(false);
});

it("sees an interleaved buffer marked edited", () => {
  const data = new InterleavedBuffer(
    new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    3,
  );
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new InterleavedBufferAttribute(data, 3, 0),
  );
  const snapshot = snapshotGeometry(geometry);
  data.array[0] = 2;
  data.needsUpdate = true;
  expect(matchesGeometry(snapshot, geometry)).toBe(false);
});
