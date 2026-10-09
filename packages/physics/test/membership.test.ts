import { afterEach, expect, it } from "vitest";
import { Group, Matrix4, Scene, Vector3 } from "three";
import {
  BoxCollider,
  FixedJoint,
  PhysicsWorld,
  RevoluteJoint,
  RigidBody,
  Trigger,
  type Joint,
  type PhysicsVelocity,
  type PhysicsWorldOptions,
} from "../src/index.js";
import {
  authoredJointReading,
  authoredVelocity,
  build,
} from "../src/backend.js";

type Member = RigidBody | Joint | Trigger;

class RecordingWorld extends PhysicsWorld {
  readonly events: string[] = [];
  freed = 0;
  failPrepare = false;
  rejectTeleport = false;
  /** Steps left before a step throws; negative never throws. */
  failStepIn = -1;
  /** Each body's simulated velocity, from its authored one as it joins. */
  readonly velocities = new Map<RigidBody, PhysicsVelocity>();
  constructor(options: PhysicsWorldOptions) {
    super(options, () => false);
  }
  /** Samples overlaps, as a backend step would. */
  overlap(trigger: Trigger, ...bodies: RigidBody[]): void {
    this.interactions.replace(new Map([[trigger, new Set(bodies)]]), new Map());
  }
  protected add(object: Member): void {
    this.events.push(`add ${object.name}`);
    if (object instanceof RigidBody)
      this.velocities.set(object, authoredVelocity(object));
  }
  protected remove(object: Member): void {
    this.events.push(`remove ${object.name}`);
  }
  protected readVelocity(object: RigidBody): PhysicsVelocity {
    const { linear, angular } = this.velocity(object);
    return { linear: linear.clone(), angular: angular.clone() };
  }
  protected writeVelocity(
    object: RigidBody,
    value: Partial<PhysicsVelocity>,
  ): void {
    this.events.push(`velocity ${object.name}`);
    if (value.linear) this.velocity(object).linear.copy(value.linear);
  }
  private velocity(object: RigidBody): PhysicsVelocity {
    const velocity = this.velocities.get(object);
    if (!velocity) throw new Error("Missing recorded velocity");
    return velocity;
  }
  protected writeImpulse(object: RigidBody): void {
    this.events.push(`impulse ${object.name}`);
  }
  protected writePoses(bodies: ReadonlySet<RigidBody>): void {
    if (this.rejectTeleport) throw new Error("teleport rejected");
    this.events.push(`teleport ${[...bodies].map(({ name }) => name)}`);
  }
  protected jointReading(object: Joint) {
    return authoredJointReading(object);
  }
  protected cast = () => null;
  protected writeTarget(): void {}
  protected writeForce(): void {}
  protected writeSleeping(): void {}
  protected prepare(): void {
    if (this.failPrepare) throw new Error("prepare failed");
  }
  protected step(): void {
    if (this.failStepIn-- === 0) throw new Error("step failed");
    this.events.push("step");
  }
  protected restore(): void {
    this.events.push("restore");
  }
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

async function create(scene: Scene): Promise<RecordingWorld> {
  const world = new RecordingWorld({ scene });
  worlds.push(world);
  return build(world);
}

it("simulates the bodies and joints under the scene, joints after their bodies", async () => {
  const scene = new Scene();
  const base = body("base");
  const arm = body("arm");
  scene.add(hinge("anchor", null, base), hinge("elbow", base, arm), base, arm);
  body("outside");

  const world = await create(scene);

  expect(world.events).toEqual([
    "add base",
    "add arm",
    "add anchor",
    "add elbow",
  ]);
  expect(arm.world).toBe(world);
});

it("adds objects entering the scene and removes leaving ones, joints first", async () => {
  const scene = new Scene();
  const base = body("base");
  const arm = body("arm");
  scene.add(base);
  const world = await create(scene);
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

it("rejects a joint under the scene whose body is outside it", async () => {
  const scene = new Scene();
  const base = body("base");
  scene.add(base, hinge("elbow", base, body("arm")));

  await expect(create(scene)).rejects.toThrow(
    "Joint elbow connects a body outside the world's scene",
  );
  expect(base.world).toBeUndefined();
});

it("keeps authored state outside every scene and rejects simulation commands there", async () => {
  const scene = new Scene();
  const outside = body("outside");
  const world = await create(scene);

  outside.setVelocity({ linear: new Vector3(1, 0, 0) });
  expect(outside.getVelocity().linear.x).toBe(1);
  expect(() => outside.applyImpulse(new Vector3(0, 1, 0))).toThrow(
    "outside every world's scene; add it under the scene of a built world",
  );
  expect(world.events).toEqual([]);

  scene.add(outside);
  outside.setVelocity({ linear: new Vector3(2, 0, 0) });
  outside.applyImpulse(new Vector3(0, 1, 0));
  expect(world.events).toEqual([
    "add outside",
    "velocity outside",
    "impulse outside",
  ]);
  expect(() => outside.applyImpulse(new Vector3(NaN, 0, 0))).toThrow("finite");
  expect(() => world.applyForce(body("stray"), new Vector3())).toThrow(
    "Physics object stray is outside the world's scene",
  );
});

it("simulates an object in one world at a time", async () => {
  const left = new Scene();
  const right = new Scene();
  const a = body("a");
  const b = body("b");
  left.add(a);
  right.add(b);

  const first = await create(left);
  const second = await create(right);
  expect(a.world).toBe(first);
  expect(b.world).toBe(second);

  const nested = new Scene();
  nested.add(body("c"));
  left.add(nested);
  await create(nested);
  expect(() => first.update(0)).toThrow("under the scenes of two worlds");

  first.dispose();
  expect(a.world).toBeUndefined();
});

it("delivers removal events to prepared members and frees a world its listener disposed once", async () => {
  const scene = new Scene();
  const zone = new Trigger();
  const ball = body("ball");
  scene.add(zone, ball);
  const world = await create(scene);
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

it("authors poses and velocities before building and rejects simulation commands", async () => {
  const body = new RigidBody();
  body.setVelocity({ linear: new Vector3(2, 0, 0) });
  body.teleport(new Matrix4().makeTranslation(1, 2, 3));
  expect(body.position.toArray()).toEqual([1, 2, 3]);
  expect(body.getVelocity().linear.x).toBe(2);
  const joint = new RevoluteJoint({ body0: null, body1: body });
  expect(joint.getState().position).toBeCloseTo(0);
  expect(() => body.applyImpulse(new Vector3(3, 0, 0))).toThrow(
    "outside every world's scene",
  );
  expect(() => body.setKinematicTarget(new Matrix4())).toThrow("kinematic");
  const hand = new RigidBody({ bodyType: "kinematic" });
  hand.setKinematicTarget(new Matrix4().makeTranslation(4, 5, 6));
  expect(hand.position.toArray()).toEqual([4, 5, 6]);
  expect(() =>
    new RigidBody({ bodyType: "static" }).setVelocity({
      linear: new Vector3(),
    }),
  ).toThrow("dynamic body");
  expect(() => body.teleport(new Matrix4().makeScale(2, 2, 2))).toThrow(
    "unit scale",
  );
});

it("teleports an authored assembly before a world exists", async () => {
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

it("undoes a failed join so objects join afresh with their authored state", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball").setVelocity({ linear: new Vector3(2, 0, 0) });
  scene.add(ball);
  world.failPrepare = true;
  expect(() => world.update(0)).toThrow("prepare failed");
  expect(() => ball.applyImpulse(new Vector3())).toThrow("prepare failed");
  expect(world.events).toEqual([
    "add ball",
    "remove ball",
    "add ball",
    "remove ball",
  ]);
  expect(ball.world).toBeUndefined();

  world.failPrepare = false;
  expect(ball.getVelocity().linear.x).toBe(2);
  expect(ball.world).toBe(world);
});

it("lets before-step callbacks command objects added after the previous step", async () => {
  const scene = new Scene();
  const world = await create(scene);
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

it.each(["before", "after"] as const)(
  "joins an object a command reaches right after it was added, %s the world's update",
  async (order) => {
    const scene = new Scene();
    const world = await create(scene);
    world.events.length = 0;
    const load = () => {
      const ball = body("ball");
      scene.add(ball);
      ball.applyImpulse(new Vector3(0, 1, 0));
    };
    if (order === "before") load();
    world.update(world.fixedDelta);
    if (order === "after") load();
    expect(world.events).toEqual(
      order === "before"
        ? ["add ball", "impulse ball", "step"]
        : ["step", "add ball", "impulse ball"],
    );
  },
);

it("joins objects that step callbacks add and command", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball");
  world.onAfterStep(() => {
    scene.add(ball);
    ball.applyImpulse(new Vector3(0, 1, 0));
  });
  world.update(world.fixedDelta);
  expect(world.events).toEqual(["step", "add ball", "impulse ball"]);
});

it("reads a trigger's overlaps right after it was added", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const zone = new Trigger();
  scene.add(zone);
  expect(zone.getOverlappingBodies()).toEqual([]);
  expect(zone.world).toBe(world);
});

it("teleports along joints that wait to join and not along those that left", async () => {
  const scene = new Scene();
  const a = body("a");
  const b = body("b");
  b.position.x = 1;
  scene.add(a, b);
  const world = await create(scene);
  const weld = new FixedJoint({ body0: a, body1: b });
  weld.name = "weld";
  scene.add(weld);
  world.events.length = 0;
  a.teleport(new Matrix4().makeTranslation(0, 10, 0));
  expect(world.events).toEqual(["add weld", "teleport a,b"]);
  expect(b.position.toArray()).toEqual([1, 10, 0]);

  weld.removeFromParent();
  world.events.length = 0;
  a.teleport(new Matrix4().makeTranslation(0, 20, 0));
  expect(world.events).toEqual(["remove weld", "teleport a"]);
  expect(b.position.y).toBe(10);
});

it("puts the scene back when the backend rejects a teleport", async () => {
  const scene = new Scene();
  const a = body("a");
  scene.add(a);
  const world = await create(scene);
  world.rejectTeleport = true;
  expect(() => a.teleport(new Matrix4().makeTranslation(0, 5, 0))).toThrow(
    "teleport rejected",
  );
  expect(a.position.toArray()).toEqual([0, 0, 0]);
});

it("finishes leaving and disposal when a simulated velocity is not finite", async () => {
  const scene = new Scene();
  const a = body("a");
  const b = body("b");
  scene.add(a, b);
  const world = await create(scene);
  for (const object of [a, b])
    world.velocities.get(object)?.linear.set(NaN, 0, 0);

  a.removeFromParent();
  world.update(0);
  expect(world.events.at(-1)).toBe("remove a");
  expect(a.world).toBeUndefined();
  expect(a.getVelocity().linear.x).toBeNaN();

  world.dispose();
  expect(world.disposed).toBe(true);
  expect(world.freed).toBe(1);
  expect(b.world).toBeUndefined();
  expect(b.getVelocity().linear.x).toBeNaN();
});

it("delivers the events listeners queue while another listener throws", async () => {
  const scene = new Scene();
  const zone = new Trigger();
  const a = body("a");
  const b = body("b");
  scene.add(zone, a, b);
  const world = await create(scene);
  const exits: RigidBody[] = [];
  zone.addEventListener("exit", ({ body }) => exits.push(body));
  zone.addEventListener("enter", ({ body }) => {
    if (body !== a) throw new Error("listener failed");
    b.removeFromParent();
    world.raycast(new Vector3(), new Vector3(0, -1, 0), 1);
  });
  world.overlap(zone, a, b);
  expect(() => world.update(0)).toThrow("listener failed");
  expect(exits).toEqual([b]);
  expect(zone.getOverlappingBodies()).toEqual([a]);
});

it("syncs before resetting, so leavers stay as they are and newcomers join", async () => {
  const scene = new Scene();
  const gone = body("gone");
  scene.add(gone);
  const world = await create(scene);
  world.update(world.fixedDelta);
  gone.removeFromParent();
  gone.position.x = 42;
  const added = body("added");
  scene.add(added);
  world.events.length = 0;
  world.reset();
  expect(world.events).toEqual(["remove gone", "add added", "restore"]);
  expect(gone.world).toBeUndefined();
  expect(gone.position.x).toBe(42);
  expect(world.time).toBe(0);
});

it("keeps the time of the steps a failed update completed and drops the rest", async () => {
  const world = await create(new Scene());
  const step = world.fixedDelta;
  world.update(step / 2);
  world.failStepIn = 1;
  expect(() => world.update(1.5 * step)).toThrow("step failed");
  expect(world.time).toBe(step);
  world.update(0);
  expect(world.time).toBe(step);
  world.update(step / 4);
  expect(world.time).toBe(step);
});

it("runs step callbacks added during a step from the next step", async () => {
  const world = await create(new Scene());
  const calls: string[] = [];
  world.onBeforeStep(() => {
    calls.push("before");
    world.onBeforeStep(() => calls.push("late before"));
  });
  world.onAfterStep(() => {
    calls.push("after");
    world.onAfterStep(() => calls.push("late after"));
  });
  world.update(world.fixedDelta);
  expect(calls).toEqual(["before", "after"]);
  calls.length = 0;
  world.update(world.fixedDelta);
  expect(calls).toEqual(["before", "late before", "after", "late after"]);
});

it("hands an object to a new world once the old world's scene no longer holds it", async () => {
  const left = new Scene();
  const right = new Scene();
  const ball = body("ball");
  left.add(ball);
  const first = await create(left);
  const second = await create(right);
  right.add(ball);
  ball.applyImpulse(new Vector3(0, 1, 0));
  expect(ball.world).toBe(second);
  expect(first.events.at(-1)).toBe("remove ball");
  expect(second.events.slice(-2)).toEqual(["add ball", "impulse ball"]);
});

it("allows one live world per scene and exposes its gravity", async () => {
  const scene = new Scene();
  const world = await create(scene);
  expect(world.gravity).toEqual([0, -9.81, 0]);
  await expect(create(scene)).rejects.toThrow("already has a physics world");
  // The rejected world's disposal leaves the scene to the live one.
  await expect(create(scene)).rejects.toThrow("already has a physics world");
  world.dispose();
  expect((await create(scene)).disposed).toBe(false);
});

it("marks physics objects and keeps their world read-only", () => {
  const ball = body("ball");
  const objects = [
    ball,
    new Trigger(),
    new BoxCollider(),
    new RevoluteJoint({ body0: null, body1: ball }),
  ];
  expect(objects.map((object) => object.isPhysicsObject)).toEqual([
    true,
    true,
    true,
    true,
  ]);
  expect(new Group()).not.toHaveProperty("isPhysicsObject");
  for (const object of objects.slice(0, 3))
    expect(
      Object.getOwnPropertyDescriptor(Object.getPrototypeOf(object), "world")
        ?.set,
    ).toBeUndefined();
});

it("acts on a body removed from the scene as authored once its old world lets it go", async () => {
  const scene = new Scene();
  const ball = body("ball");
  const hand = new RigidBody({ bodyType: "kinematic", colliders: false });
  hand.name = "hand";
  scene.add(ball, hand);
  const world = await create(scene);
  world.velocities.get(ball)?.linear.set(3, 0, 0);
  world.events.length = 0;

  ball.removeFromParent();
  hand.removeFromParent();
  ball.teleport(new Matrix4().makeTranslation(0, 5, 0));
  hand.setKinematicTarget(new Matrix4().makeTranslation(0, 7, 0));
  expect(world.events).toEqual(["remove ball", "remove hand"]);
  expect([ball.world, hand.world]).toEqual([undefined, undefined]);
  expect([ball.position.y, hand.position.y]).toEqual([5, 7]);
  expect(ball.getVelocity().linear.x).toBe(3);
});

it("clones a body waiting under the scene without joining it", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball").setVelocity({ linear: new Vector3(2, 0, 0) });
  scene.add(ball);
  const copy = ball.clone();
  expect(copy.getVelocity().linear.x).toBe(2);
  expect(ball.world).toBeUndefined();
  expect(world.events).toEqual([]);
});

it("raycasts excluding bodies outside the world", async () => {
  const world = await create(new Scene());
  expect(
    world.raycast(new Vector3(), new Vector3(0, -1, 0), 1, {
      excludeBodies: [body("stray")],
    }),
  ).toBeNull();
});

it("says a claim failed while the previous world released the object", async () => {
  const left = new Scene();
  const right = new Scene();
  const ball = body("ball");
  left.add(ball);
  const first = await create(left);
  const second = await create(right);
  right.add(ball);
  first.failPrepare = true;
  let error: unknown;
  try {
    second.update(0);
  } catch (caught) {
    error = caught;
  }
  expect(error).toMatchObject({
    message: "Releasing physics object ball from its previous world failed",
    cause: { message: "prepare failed" },
  });
});

it("takes an object a listener of its previous world already had it take", async () => {
  const left = new Scene();
  const right = new Scene();
  const zone = new Trigger();
  const ball = body("ball");
  left.add(zone, ball);
  const first = await create(left);
  const second = await create(right);
  first.overlap(zone, ball);
  first.update(0);
  zone.addEventListener("exit", () =>
    second.raycast(new Vector3(), new Vector3(0, -1, 0), 1),
  );
  right.add(ball);
  second.update(0);
  expect(ball.world).toBe(second);
  expect(second.events).toEqual(["add ball"]);
});

it("rejects world commands on an object that left the world's scene", async () => {
  const left = new Scene();
  const right = new Scene();
  const a = body("a");
  const b = body("b");
  left.add(a, b);
  const first = await create(left);
  await create(right);
  a.removeFromParent();
  right.add(b);
  for (const object of [a, b])
    expect(() => first.applyImpulse(object, new Vector3())).toThrow(
      "outside the world's scene",
    );
  expect(first.events).toEqual(["add a", "add b", "remove a", "remove b"]);
});

it("skips step callbacks an earlier callback removed", async () => {
  const world = await create(new Scene());
  const calls: string[] = [];
  let offBefore = () => {};
  let offAfter = () => {};
  world.onBeforeStep(() => offBefore());
  offBefore = world.onBeforeStep(() => calls.push("before"));
  world.onAfterStep(() => offAfter());
  offAfter = world.onAfterStep(() => calls.push("after"));
  world.update(world.fixedDelta);
  expect(calls).toEqual([]);
});

it("delivers the events leavers queued when joining fails", async () => {
  const scene = new Scene();
  const zone = new Trigger();
  const ball = body("ball");
  scene.add(zone, ball);
  const world = await create(scene);
  world.overlap(zone, ball);
  world.update(0);
  const exits: RigidBody[] = [];
  zone.addEventListener("exit", ({ body }) => exits.push(body));
  ball.removeFromParent();
  scene.add(body("late"));
  world.failPrepare = true;
  expect(() => world.update(0)).toThrow("prepare failed");
  expect(exits).toEqual([ball]);
});

it("leaves objects authored while a world is still building", async () => {
  const scene = new Scene();
  const ball = body("ball");
  scene.add(ball);
  const building = create(scene);
  ball.setVelocity({ linear: new Vector3(1, 0, 0) });
  expect(ball.world).toBeUndefined();
  const world = await building;
  expect(ball.world).toBe(world);
  expect(ball.getVelocity().linear.x).toBe(1);
});

it("drops the unspent time of a failed update", async () => {
  const scene = new Scene();
  const world = await create(scene);
  scene.add(body("ball"));
  world.failPrepare = true;
  expect(() => world.update(3 * world.fixedDelta)).toThrow("prepare failed");
  world.failPrepare = false;
  world.update(0);
  expect(world.time).toBe(0);
});

it("clones a body removed since the last sync with its simulated velocity", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball");
  scene.add(ball);
  world.update(0);
  ball.setVelocity({ linear: new Vector3(3, 0, 0) });
  ball.removeFromParent();
  const copy = ball.clone();
  expect(ball.world).toBeUndefined();
  expect(copy.getVelocity().linear.x).toBe(3);
});
