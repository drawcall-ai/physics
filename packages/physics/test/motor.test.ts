import { afterEach, expect, test } from "vitest";
import { Group, Matrix4, Scene, Vector3 } from "three";
import {
  CapsuleMotor,
  PhysicsWorld,
  RigidBody,
  type CapsuleContact,
  type CapsuleShape,
  type JointReading,
  type PhysicsVelocity,
} from "../src/index.js";

class QueryWorld extends PhysicsWorld {
  contact: CapsuleContact | undefined;
  failure: Error | undefined;
  casts = 0;
  readonly requests: { shape: CapsuleShape; pose: Matrix4 }[] = [];

  constructor() {
    super({ scene: new Scene() }, () => false);
  }
  protected capsuleContactQuery(shape: CapsuleShape, pose: Matrix4) {
    this.requests.push({ shape: { ...shape }, pose: pose.clone() });
    if (this.failure) throw this.failure;
    return this.contact ? [this.contact] : [];
  }
  protected capsuleCastQuery() {
    this.casts++;
    return null;
  }
  protected prepare(): void {}
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
function setup() {
  const world = new QueryWorld();
  worlds.push(world);
  return world;
}
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

test("rejects invalid dimensions and native body ownership before accepting a motor", () => {
  const world = setup();
  const object = new Group();
  for (const radius of [0, -1, Infinity, NaN])
    expect(() => new CapsuleMotor(world, object, radius, 1.8)).toThrow(
      "radius",
    );
  for (const height of [0.7, -1, Infinity, NaN])
    expect(() => new CapsuleMotor(world, object, 0.35, height)).toThrow(
      "height",
    );
  expect(() => new CapsuleMotor(world, new RigidBody(), 0.35, 1.8)).toThrow(
    "visual object",
  );
  const motor = new CapsuleMotor(world, object, 0.35, 1.8);
  for (const height of [0.7, -1, Infinity, NaN]) {
    expect(() => {
      motor.height = height;
    }).toThrow("height");
    expect(() => motor.resize(height)).toThrow("height");
    expect(motor.height).toBe(1.8);
  }
  expect(world.requests).toEqual([]);
});

test("growth checks the candidate capsule and preserves height on obstruction or backend failure", () => {
  const world = setup();
  const object = new Group();
  object.position.y = 3;
  const motor = new CapsuleMotor(world, object, 0.35, 1.1);
  const obstacle = new RigidBody({ bodyType: "static" });
  world.contact = {
    body: obstacle,
    object: obstacle,
    point: new Vector3(0, 4.5, 0),
    normal: new Vector3(0, -1, 0),
    distance: -0.2,
  };
  expect(motor.resize(1.8)).toBe(false);
  expect(motor.height).toBe(1.1);
  expect(world.requests[0]?.shape.height).toBeCloseTo(1.1);
  expect(world.requests[0]?.pose.elements[13]).toBeCloseTo(3.9);
  world.contact = undefined;
  world.failure = new Error("Native query failed");
  expect(() => motor.resize(1.8)).toThrow("Native query failed");
  expect(motor.height).toBe(1.1);
  world.failure = undefined;
  expect(motor.resize(1.8)).toBe(true);
  expect(motor.height).toBe(1.8);
  expect(motor.position.y).toBe(3);
});

test("invalid translations and ground distances never corrupt the staged or visual pose", () => {
  const world = setup();
  const object = new Group();
  const motor = new CapsuleMotor(world, object, 0.35, 1.8);
  for (const displacement of [
    new Vector3(NaN, 0, 0),
    new Vector3(0, Infinity, 0),
    new Vector3(Number.MAX_VALUE, Number.MAX_VALUE, 0),
  ]) {
    expect(() => motor.move(displacement, false)).toThrow("finite");
    expect(() => motor.carry(displacement, new RigidBody())).toThrow("finite");
  }
  for (const distance of [-1, Infinity, NaN])
    expect(() => motor.ground(distance)).toThrow("distance");
  expect(motor.position.toArray()).toEqual([0, 0, 0]);
  expect(world.casts).toBe(0);
  motor.position.x = NaN;
  expect(() => motor.move(new Vector3(), false)).toThrow("finite");
  expect(() => motor.commit()).toThrow("finite");
  expect(object.position.toArray()).toEqual([0, 0, 0]);
});

test("stages world-space movement and commits through the current parent without changing visual rotation or scale", () => {
  const world = setup();
  const parent = new Group();
  parent.position.set(4, 2, -3);
  parent.rotation.y = 0.7;
  parent.scale.set(-2, 3, 2);
  const object = new Group();
  object.position.set(1, 0, 2);
  object.rotation.x = 0.3;
  object.scale.setScalar(0.5);
  object.updateMatrix();
  object.matrixAutoUpdate = false;
  world.scene.add(parent.add(object));
  const motor = new CapsuleMotor(world, object, 0.35, 1.8);
  const local = object.position.clone();
  const rotation = object.quaternion.clone();
  const scale = object.scale.clone();
  const displacement = new Vector3(1, 0, -2);
  const destination = motor.position.clone().add(displacement);
  expect(motor.move(displacement, false)).toEqual([]);
  expect(object.position.equals(local)).toBe(true);
  expect(displacement.toArray()).toEqual([1, 0, -2]);
  parent.position.x += 5;
  parent.rotation.y -= 0.2;
  motor.commit();
  expect(
    object.getWorldPosition(new Vector3()).distanceTo(destination),
  ).toBeLessThan(1e-10);
  expect(object.quaternion.equals(rotation)).toBe(true);
  expect(object.scale.equals(scale)).toBe(true);
  const before = object.position.clone();
  parent.scale.x = 0;
  expect(() => motor.commit()).toThrow("Capsule motor parent");
  expect(object.position.equals(before)).toBe(true);
});
