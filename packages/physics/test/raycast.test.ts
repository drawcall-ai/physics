import { expect, it } from "vitest";
import { Vector3 } from "three";
import { rayDirection } from "../src/raycast.js";

const origin = new Vector3(-3, 0, 0);
const direction = new Vector3(1, 0, 0);

it("rejects invalid ray inputs", () => {
  for (const invalid of [
    new Vector3(),
    new Vector3(NaN, 0, 0),
    new Vector3(Infinity, 0, 0),
  ])
    expect(() => rayDirection(origin, invalid, 5, {})).toThrow();
  for (const distance of [-1, NaN, Infinity])
    expect(() => rayDirection(origin, direction, distance, {})).toThrow();
  expect(() =>
    rayDirection(new Vector3(NaN, 0, 0), direction, 5, {}),
  ).toThrow();
});
