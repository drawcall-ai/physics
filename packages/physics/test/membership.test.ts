import { afterEach, expect, it } from "vitest";
import { Group, Matrix4, Scene, Vector3 } from "three";
import {
  PhysicsWorld,
  RevoluteJoint,
  RigidBody,
  Trigger,
  type Joint,
  type PhysicsVelocity,
} from "../src/index.js";
import { authoredJointReading, build } from "../src/backend.js";

type Member = RigidBody | Joint | Trigger;

class RecordingWorld extends PhysicsWorld {
  readonly events: string[] = [];
  freed = 0;
  failPrepare = false;
  /** Samples one overlap, as a backend step would. */
  overlap(trigger: Trigger, body: RigidBody): void {
    this.interactions.replace(new Map([[trigger, new Set([body])]]), new Map());
  }
  private readonly velocities = new Map<RigidBody, PhysicsVelocity>();
  protected add(object: Member): void {
    this.events.push(`add ${object.name}`);
    if (object instanceof RigidBody)
      this.velocities.set(object, object.getVelocity());
  }
  protected remove(object: Member): void {
    this.events.push(`remove ${object.name}`);
  }
  getVelocity(object: RigidBody): PhysicsVelocity {
    const { linear, angular } = this.velocity(object);
    return { linear: linear.clone(), angular: angular.clone() };
  }
  setVelocity(object: RigidBody, value: Partial<PhysicsVelocity>): void {
    this.events.push(`velocity ${object.name}`);
    if (value.linear) this.velocity(object).linear.copy(value.linear);
  }
  private velocity(object: RigidBody): PhysicsVelocity {
    const velocity = this.velocities.get(object);
    if (!velocity) throw new Error("Missing recorded velocity");
    return velocity;
  }
  applyImpulse(object: RigidBody): void {
    this.events.push(`impulse ${object.name}`);
  }
  readJoint(object: Joint) {
    return authoredJointReading(object);
  }
  protected cast = () => null;
  teleport(): void {}
  setKinematicTarget(): void {}
  applyForce(): void {}
  wake(): void {}
  sleep(): void {}
  protected prepare(): void {
    if (this.failPrepare) throw new Error("prepare failed");
  }
  protected step(): void {}
  protected restore(): void {}
  protected free(): void {
    this.freed++;
  }
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

function create(scene: Scene): RecordingWorld {
  const world = build(new RecordingWorld({ scene }));
  worlds.push(world);
  return world;
}

it("simulates the bodies and joints under the scene, joints after their bodies", () => {
  const scene = new Scene();
  const base = body("base");
  const arm = body("arm");
  scene.add(hinge("anchor", null, base), hinge("elbow", base, arm), base, arm);
  body("outside");

  const world = create(scene);

  expect(world.events).toEqual([
    "add base",
    "add arm",
    "add anchor",
    "add elbow",
  ]);
  expect(arm.world).toBe(world);
});

it("adds objects entering the scene and removes leaving ones, joints first", () => {
  const scene = new Scene();
  const base = body("base");
  const arm = body("arm");
  scene.add(base);
  const world = create(scene);
  world.events.length = 0;

  const elbow = hinge("elbow", base, arm);
  scene.add(arm, elbow);
  world.update(0);
  expect(world.events).toEqual(["add arm", "add elbow"]);

  world.events.length = 0;
  arm.removeFromParent();
  elbow.removeFromParent();
  world.update(0);
  expect(world.events).toEqual(["remove elbow", "remove arm"]);
});

it("rejects a joint under the scene whose body is outside it", () => {
  const scene = new Scene();
  const base = body("base");
  scene.add(base, hinge("elbow", base, body("arm")));

  expect(() => create(scene)).toThrow(
    "Joint connects a body outside the world's scene",
  );
  expect(base.world).toBeUndefined();
});

it("keeps authored state outside a scene and rejects simulation commands there", () => {
  const scene = new Scene();
  const outside = body("outside");
  const world = create(scene);

  outside.setVelocity({ linear: new Vector3(1, 0, 0) });
  expect(outside.getVelocity().linear.x).toBe(1);
  expect(() => outside.applyImpulse(new Vector3(0, 1, 0))).toThrow(
    "has not joined a world yet",
  );
  expect(world.events).toEqual([]);

  scene.add(outside);
  expect(() => outside.applyImpulse(new Vector3(0, 1, 0))).toThrow(
    "has not joined a world yet",
  );
  world.update(0);
  world.events.length = 0;
  outside.setVelocity({ linear: new Vector3(2, 0, 0) });
  outside.applyImpulse(new Vector3(0, 1, 0));
  expect(world.events).toEqual(["velocity outside", "impulse outside"]);
});

it("simulates an object in one world at a time", () => {
  const left = new Scene();
  const right = new Scene();
  const a = body("a");
  const b = body("b");
  left.add(a);
  right.add(b);

  const first = create(left);
  const second = create(right);
  expect(a.world).toBe(first);
  expect(b.world).toBe(second);

  const nested = new Scene();
  nested.add(body("c"));
  left.add(nested);
  create(nested);
  expect(() => first.update(0)).toThrow("already simulated by another world");

  first.dispose();
  expect(a.world).toBeUndefined();
});

it("delivers removal events to prepared members and frees a world its listener disposed once", () => {
  const scene = new Scene();
  const zone = new Trigger();
  const ball = body("ball");
  scene.add(zone, ball);
  const world = create(scene);
  world.overlap(zone, ball);
  world.update(0);
  expect(zone.getOverlappingBodies()).toEqual([ball]);

  const seen: unknown[] = [];
  zone.addEventListener("exit", (event) => {
    seen.push(event.body, zone.world);
    world.dispose();
  });
  world.events.length = 0;
  ball.removeFromParent();
  expect(() => zone.getOverlappingBodies()).toThrow("disposed");
  expect(seen).toEqual([ball, world]);
  expect(world.events).toEqual(["remove ball"]);
  expect(world.freed).toBe(1);
});

it("authors poses and velocities before building and rejects simulation commands", () => {
  const body = new RigidBody();
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.teleport(new Matrix4().makeTranslation(1, 2, 3));
  expect(body.position.toArray()).toEqual([1, 2, 3]);
  expect(body.getVelocity().linear.x).toBe(2);
  const joint = new RevoluteJoint({ body0: null, body1: body });
  expect(joint.getState().position).toBeCloseTo(0);
  expect(() => body.applyImpulse(new Vector3(3, 0, 0))).toThrow(
    "has not joined a world yet",
  );
  expect(() => body.setKinematicTarget(new Matrix4())).toThrow("kinematic");
  const hand = new RigidBody({ bodyType: "kinematic" });
  hand.setKinematicTarget(new Matrix4().makeTranslation(4, 5, 6));
  expect(hand.position.toArray()).toEqual([4, 5, 6]);
  expect(() => body.applyForce(new Vector3(NaN, 0, 0))).toThrow("finite");
  expect(() =>
    new RigidBody({ bodyType: "static" }).setVelocity({
      linear: new Vector3(),
    }),
  ).toThrow("dynamic body");
  expect(() => body.teleport(new Matrix4().makeScale(2, 2, 2))).toThrow(
    "unit scale",
  );
});

it("teleports an authored assembly before a world exists", () => {
  const root = new RigidBody();
  const child = new RigidBody();
  child.position.x = 2;
  const joint = new RevoluteJoint({ body0: root, body1: child });
  joint.position.x = 1;
  new Group().add(root, child, joint);
  root.teleport(new Matrix4().makeTranslation(0, 5, 0));
  expect(child.position.toArray()).toEqual([2, 5, 0]);
  expect(joint.getState().position).toBeCloseTo(0);
});

it("undoes a failed join so objects join afresh with their authored state", () => {
  const scene = new Scene();
  const world = create(scene);
  const ball = body("ball");
  scene.add(ball);
  world.failPrepare = true;
  expect(() => world.update(0)).toThrow("prepare failed");
  expect(world.events).toEqual(["add ball", "remove ball"]);
  expect(ball.world).toBeUndefined();

  world.failPrepare = false;
  ball.setVelocity({ linear: new Vector3(2, 0, 0) });
  world.update(0);
  expect(ball.world).toBe(world);
  expect(ball.getVelocity().linear.x).toBe(2);
});

it("lets before-step callbacks command objects added after the previous step", () => {
  const scene = new Scene();
  const world = create(scene);
  const spawned = body("spawned");
  const seen: unknown[] = [];
  world.onAfterStep(() => {
    if (!spawned.parent) scene.add(spawned);
  });
  world.onBeforeStep(() => {
    if (spawned.parent) seen.push(spawned.world);
  });
  world.update(2 * world.fixedDelta);
  expect(seen).toEqual([world]);
});
