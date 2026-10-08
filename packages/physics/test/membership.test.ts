import { afterEach, expect, it } from "vitest";
import { Scene, Vector3 } from "three";
import {
  RevoluteJoint,
  RigidBody,
  SteppedWorld,
  authoredVelocity,
  buildRooted,
  sceneJointReading,
  setAuthoredVelocity,
  worldOf,
  type Joint,
  type PhysicsVelocity,
  type Trigger,
} from "../src/index.js";

type Member = RigidBody | Joint | Trigger;

class RecordingWorld extends SteppedWorld {
  readonly events: string[] = [];
  protected add(object: Member): void {
    this.events.push(`add ${object.name}`);
  }
  protected remove(object: Member): void {
    this.events.push(`remove ${object.name}`);
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    return authoredVelocity(object);
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.events.push(`velocity ${object.name}`);
    setAuthoredVelocity(object, value);
  }
  applyImpulse(object: RigidBody): void {
    this.events.push(`impulse ${object.name}`);
  }
  readJoint(object: Joint) {
    return sceneJointReading(object);
  }
  protected cast = () => null;
  teleport(): void {}
  setKinematicTarget(): void {}
  applyForce(): void {}
  wake(): void {}
  sleep(): void {}
  protected prepare(): void {}
  protected step(): void {}
  protected restore(): void {}
  protected free(): void {}
}

function body(name: string): RigidBody {
  const result = new RigidBody({ colliders: false });
  result.name = name;
  return result;
}

function hinge(name: string, body0: RigidBody | null, body1: RigidBody) {
  const result = new RevoluteJoint({ body0, body1 });
  result.name = name;
  return result;
}

const worlds: RecordingWorld[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});

async function build(root: Scene): Promise<RecordingWorld> {
  const world = await buildRooted(
    root,
    async () => new RecordingWorld(root, {}),
  );
  worlds.push(world);
  return world;
}

it("simulates the bodies and joints under the root, joints after their bodies", async () => {
  const root = new Scene();
  const base = body("base");
  const arm = body("arm");
  root.add(hinge("anchor", null, base), hinge("elbow", base, arm), base, arm);
  body("outside");

  const world = await build(root);

  expect(world.events).toEqual([
    "add base",
    "add arm",
    "add anchor",
    "add elbow",
  ]);
  expect(worldOf(arm)).toBe(world);
});

it("adds objects entering the root and removes leaving ones, joints first", async () => {
  const root = new Scene();
  const base = body("base");
  const arm = body("arm");
  root.add(base);
  const world = await build(root);
  world.events.length = 0;

  const elbow = hinge("elbow", base, arm);
  root.add(arm, elbow);
  world.update(0);
  expect(world.events).toEqual(["add arm", "add elbow"]);

  world.events.length = 0;
  arm.removeFromParent();
  elbow.removeFromParent();
  world.update(0);
  expect(world.events).toEqual(["remove elbow", "remove arm"]);
});

it("rejects a joint under the root whose body is outside it", async () => {
  const root = new Scene();
  const base = body("base");
  root.add(base, hinge("elbow", base, body("arm")));

  await expect(build(root)).rejects.toThrow(
    "Joint connects a body outside the world's root",
  );
  expect(worldOf(base)).toBeUndefined();
});

it("keeps authored state outside a root and rejects simulation commands there", async () => {
  const root = new Scene();
  const outside = body("outside");
  const world = await build(root);

  outside.setVelocity({ linear: new Vector3(1, 0, 0) });
  expect(outside.getVelocity().linear.x).toBe(1);
  expect(() => outside.applyImpulse(new Vector3(0, 1, 0))).toThrow(
    "under a built world's root",
  );
  expect(world.events).toEqual([]);

  root.add(outside);
  outside.setVelocity({ linear: new Vector3(2, 0, 0) });
  outside.applyImpulse(new Vector3(0, 1, 0));
  expect(world.events).toEqual(["velocity outside", "impulse outside"]);
});

it("builds one world per root", async () => {
  const left = new Scene();
  const right = new Scene();
  const a = body("a");
  const b = body("b");
  left.add(a);
  right.add(b);

  const first = await build(left);
  const second = await build(right);

  expect(worldOf(a)).toBe(first);
  expect(worldOf(b)).toBe(second);
  await expect(build(left)).rejects.toThrow("already built");
  first.dispose();
  expect(worldOf(a)).toBeUndefined();
});
