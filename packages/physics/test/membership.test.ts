import { afterEach, expect, it } from "vitest";
import { Scene, Vector3 } from "three";
import {
  RevoluteJoint,
  RigidBody,
  SteppedWorld,
  authoredVelocity,
  buildRegistered,
  registry,
  sceneJointReading,
  setAuthoredVelocity,
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
  protected disposeObjects(): void {}
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

let world: RecordingWorld | undefined;
afterEach(() => {
  world?.dispose();
  world = undefined;
  registry.clear();
});

async function build(root: Scene): Promise<RecordingWorld> {
  world = await buildRegistered(root, async () => new RecordingWorld(root, {}));
  return world;
}

it("simulates the bodies under the root and the joints between them", async () => {
  const root = new Scene();
  const inside = body("inside");
  const outside = body("outside");
  root.add(inside);
  hinge("anchored", null, inside);
  hinge("straddling", inside, outside);

  const { events } = await build(root);

  expect(events).toEqual(["add inside", "add anchored"]);
  expect(registry.worldOf(inside)).toBe(world);
  expect(registry.worldOf(outside)).toBeUndefined();
});

it("adds objects entering the root and removes leaving ones without disposing them", async () => {
  const root = new Scene();
  const base = body("base");
  const arm = body("arm");
  root.add(base);
  const joint = hinge("joint", base, arm);
  const { events } = await build(root);
  events.length = 0;

  root.add(arm);
  world?.update(0);
  expect(events).toEqual(["add arm", "add joint"]);

  events.length = 0;
  arm.removeFromParent();
  world?.update(0);
  expect(events).toEqual(["remove joint", "remove arm"]);
  expect(arm.disposed).toBe(false);
  expect(joint.disposed).toBe(false);
});

it("keeps authored state outside the root and rejects simulation commands there", async () => {
  const root = new Scene();
  const outside = body("outside");
  const { events } = await build(root);

  outside.setVelocity({ linear: new Vector3(1, 0, 0) });
  expect(outside.getVelocity().linear.x).toBe(1);
  expect(() => outside.applyImpulse(new Vector3(0, 1, 0))).toThrow(
    "outside the world's root",
  );
  expect(events).toEqual([]);

  root.add(outside);
  outside.setVelocity({ linear: new Vector3(2, 0, 0) });
  outside.applyImpulse(new Vector3(0, 1, 0));
  expect(events).toEqual(["velocity outside", "impulse outside"]);
});

it("removes a disposed member once and ignores disposed outsiders", async () => {
  const root = new Scene();
  const member = body("member");
  const outside = body("outside");
  root.add(member);
  const { events } = await build(root);
  events.length = 0;

  outside.dispose();
  member.dispose();
  world?.update(0);

  expect(events).toEqual(["remove member"]);
});
