import { expect, it } from "vitest";
import { rollback } from "../src/cleanup.js";

it("rethrows the original failure after successful rollback", () => {
  const original = new Error("construction failed");
  const released: string[] = [];
  let failure: unknown;
  try {
    rollback(
      original,
      [
        () => {
          released.push("data");
        },
        () => {
          released.push("model");
        },
      ],
      "Rollback failed",
    );
  } catch (error) {
    failure = error;
  }
  expect(released).toEqual(["data", "model"]);
  expect(failure).toBe(original);
});

it("attempts every release and retains original and cleanup failures in order", () => {
  const original = new Error("construction failed");
  const first = new Error("data release failed");
  const second = new Error("model release failed");
  const released: string[] = [];
  let failure: unknown;
  try {
    rollback(
      original,
      [
        () => {
          released.push("data");
          throw first;
        },
        () => {
          released.push("model");
          throw second;
        },
        () => {
          released.push("remaining");
        },
      ],
      "Rollback failed",
    );
  } catch (error) {
    failure = error;
  }
  expect(released).toEqual(["data", "model", "remaining"]);
  expect(failure).toBeInstanceOf(AggregateError);
  if (!(failure instanceof AggregateError))
    throw new Error("Expected rollback errors");
  expect(failure.errors).toEqual([original, first, second]);
});
