import { afterEach, expect, test } from "vitest";
import { Matrix4, Scene, Vector3 } from "three";
import {
  PhysicsWorld,
  type CapsuleCastHit,
  type CapsuleContact,
  type CapsuleQueryOptions,
  type CapsuleShape,
  type JointReading,
  type PhysicsVelocity,
} from "../src/index.js";

class QueryWorld extends PhysicsWorld {
  readonly queries: string[] = [];
  prepared = 0;
  disposeOnPrepare = false;

  constructor() {
    super({ scene: new Scene() }, () => false);
  }
  protected capsuleContactQuery(
    _shape: CapsuleShape,
    _pose: Matrix4,
    _distance: number,
    _options: CapsuleQueryOptions,
  ): CapsuleContact[] {
    this.queries.push("contacts");
    return [];
  }
  protected capsuleCastQuery(): CapsuleCastHit | null {
    this.queries.push("cast");
    return null;
  }
  protected prepare(): void {
    this.prepared++;
    if (this.disposeOnPrepare) this.dispose();
  }
  protected add(): void {}
  protected remove(): void {}
  protected step(): void {}
  protected restore(): void {}
  protected free(): void {}
  protected cast() {
    return null;
  }
  protected readVelocity(): PhysicsVelocity {
    throw new Error("Unused velocity query");
  }
  protected jointReading(): JointReading {
    throw new Error("Unused joint query");
  }
  protected writeVelocity(): void {}
  protected writePoses(): void {}
  protected writeTarget(): void {}
  protected writeImpulse(): void {}
  protected writeForce(): void {}
  protected writeSleeping(): void {}
}

const worlds: QueryWorld[] = [];
function create(): QueryWorld {
  const world = new QueryWorld();
  worlds.push(world);
  return world;
}
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

const shape = { radius: 0.4, height: 1 };
const pose = new Matrix4();
const displacement = new Vector3(2, 0, 0);

test("rejects invalid capsule dimensions before preparing or querying a backend", () => {
  const world = create();
  for (const key of ["radius", "height"] as const)
    for (const value of [0, -1, NaN, Infinity, -Infinity]) {
      const invalid = { ...shape, [key]: value };
      expect(() => world.capsuleContacts(invalid, pose, 0)).toThrow(
        "dimensions",
      );
      expect(() => world.castCapsule(invalid, pose, displacement)).toThrow(
        "dimensions",
      );
    }
  expect(world.prepared).toBe(0);
  expect(world.queries).toEqual([]);
});

test("rejects non-rigid and non-finite capsule poses before backend dispatch", () => {
  const world = create();
  const shear = new Matrix4();
  shear.elements[4] = 0.3;
  const perspective = new Matrix4();
  perspective.elements[3] = 0.1;
  for (const invalid of [
    new Matrix4().makeScale(2, 1, 1),
    new Matrix4().makeScale(-1, 1, 1),
    new Matrix4().makeScale(0, 1, 1),
    new Matrix4().makeTranslation(Infinity, 0, 0),
    new Matrix4().makeTranslation(0, NaN, 0),
    shear,
    perspective,
  ]) {
    expect(() => world.capsuleContacts(shape, invalid, 1)).toThrow();
    expect(() => world.castCapsule(shape, invalid, displacement)).toThrow();
  }
  expect(world.prepared).toBe(0);
  expect(world.queries).toEqual([]);
});

test("rejects invalid contact ranges and non-finite translations", () => {
  const world = create();
  for (const distance of [-1, NaN, Infinity, -Infinity])
    expect(() => world.capsuleContacts(shape, pose, distance)).toThrow(
      "distance",
    );
  for (const axis of [0, 1, 2])
    for (const value of [NaN, Infinity, -Infinity])
      expect(() =>
        world.castCapsule(shape, pose, new Vector3().setComponent(axis, value)),
      ).toThrow("finite");
  expect(() =>
    world.castCapsule(
      shape,
      pose,
      new Vector3(Number.MAX_VALUE, Number.MAX_VALUE, 0),
    ),
  ).toThrow("finite");
  expect(world.prepared).toBe(0);
  expect(world.queries).toEqual([]);
});

test("validates both collision group masks for both query kinds", () => {
  const world = create();
  for (const key of ["membership", "filter"] as const)
    for (const value of [-1, 0.5, 65536, NaN, Infinity]) {
      const options = {
        collisionGroups: { membership: 1, filter: 1, [key]: value },
      };
      expect(() => world.capsuleContacts(shape, pose, 1, options)).toThrow(
        "16-bit",
      );
      expect(() =>
        world.castCapsule(shape, pose, displacement, options),
      ).toThrow("16-bit");
    }
  expect(world.prepared).toBe(0);
  expect(world.queries).toEqual([]);
});

test("accepts zero contact range and rigid rotations, but skips zero-length sweeps", () => {
  const world = create();
  const rotated = new Matrix4().makeRotationZ(0.7).setPosition(3, 4, 5);
  const before = rotated.clone();
  expect(
    world.capsuleContacts(shape, rotated, 0, {
      collisionGroups: { membership: 0, filter: 65535 },
    }),
  ).toEqual([]);
  expect(world.castCapsule(shape, rotated, new Vector3())).toBeNull();
  expect(world.queries).toEqual(["contacts"]);
  expect(rotated.equals(before)).toBe(true);
  expect(world.time).toBe(0);
});

test("disposed worlds reject both queries including zero movement", () => {
  const world = create();
  world.dispose();
  expect(() => world.capsuleContacts(shape, pose, 0)).toThrow("disposed");
  expect(() => world.castCapsule(shape, pose, new Vector3())).toThrow(
    "disposed",
  );
  expect(() => world.castCapsule(shape, pose, displacement)).toThrow(
    "disposed",
  );
  expect(world.prepared).toBe(0);
  expect(world.queries).toEqual([]);
});

test("disposal during refresh never queries released backend state", () => {
  for (const query of [
    (world: QueryWorld) => world.capsuleContacts(shape, pose, 1),
    (world: QueryWorld) => world.castCapsule(shape, pose, displacement),
    (world: QueryWorld) => world.castCapsule(shape, pose, new Vector3()),
  ]) {
    const world = create();
    world.disposeOnPrepare = true;
    expect(() => query(world)).toThrow("disposed");
    expect(world.queries).toEqual([]);
  }
});
