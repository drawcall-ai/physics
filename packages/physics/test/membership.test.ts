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
  type RigidBodyOptions,
} from "../src/index.js";
import { build, initialVelocity, JointBinding } from "../src/backend.js";

type Member = RigidBody | Joint | Trigger;

class RecordingWorld extends PhysicsWorld {
  readonly events: string[] = [];
  freed = 0;
  failPrepare = false;
  rejectTeleport = false;
  /** Steps left before a step throws; negative never throws. */
  failStepIn = -1;
  /** Each body's simulated velocity, from its initial one as it joins. */
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
      this.velocities.set(object, initialVelocity(object));
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
    return new JointBinding(object).read({
      angular: (body) => this.readVelocity(body).angular,
      // About the body origin, which serves as its center of mass here.
      velocityAt: (body, point) => {
        const { linear, angular } = this.readVelocity(body);
        const origin = body.getWorldPosition(new Vector3());
        return linear.add(angular.cross(point.clone().sub(origin)));
      },
    });
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
    for (const body of this.velocities.keys())
      this.velocities.set(body, initialVelocity(body));
  }
  protected free(): void {
    this.freed++;
  }
}

function body(name: string, options: RigidBodyOptions = {}): RigidBody {
  const result = new RigidBody({ ...options, colliders: false });
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

it("rejects simulation commands outside every built world's scene", async () => {
  const scene = new Scene();
  const outside = body("outside");
  const joint = hinge("hinge", null, outside);
  const trigger = new Trigger();
  new Group().add(outside, joint, trigger);
  const world = await create(scene);

  const commands = [
    () => outside.getVelocity(),
    () => outside.setVelocity({ linear: new Vector3(1, 0, 0) }),
    () => outside.teleport(new Matrix4()),
    () => outside.applyImpulse(new Vector3(0, 1, 0)),
    () => outside.wake(),
    () => joint.getState(),
    () => trigger.getOverlappingBodies(),
  ];
  for (const command of commands)
    expect(command).toThrow(
      "is not under a built world's scene; add it under one",
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

it("starts a body from its velocity option as it joins and on reset", async () => {
  const scene = new Scene();
  const ball = body("ball", { velocity: { linear: [2, 0, 0] } });
  expect(() => ball.getVelocity()).toThrow("not under a built world's scene");
  scene.add(ball);
  const world = await create(scene);
  expect(ball.getVelocity().linear.x).toBe(2);
  ball.setVelocity({ linear: new Vector3(5, 0, 0) });
  world.reset();
  expect(ball.getVelocity().linear.x).toBe(2);

  expect(() => new RigidBody({ velocity: { angular: [0, NaN, 0] } })).toThrow(
    "finite",
  );
  expect(
    () =>
      new RigidBody({ bodyType: "static", velocity: { linear: [1, 0, 0] } }),
  ).toThrow("dynamic body");
  expect(
    () =>
      new RigidBody({ bodyType: "kinematic", velocity: { linear: [0, 0, 0] } }),
  ).toThrow("dynamic body");
});
it("keeps a normalized, frozen copy of the velocity option", () => {
  const linear: [number, number, number] = [1, 2, 3];
  const ball = new RigidBody({ velocity: { linear } });
  linear[0] = 9;
  expect(ball.options.velocity).toEqual({
    linear: [1, 2, 3],
    angular: [0, 0, 0],
  });
  expect(Object.isFrozen(ball.options.velocity?.linear)).toBe(true);
  expect(initialVelocity(ball).linear.x).toBe(1);
  expect(initialVelocity(ball.clone()).linear.x).toBe(1);
  expect(new RigidBody({ bodyType: "static" }).clone().bodyType).toBe("static");
  // Untyped callers.
  expect(() =>
    Reflect.construct(RigidBody, [{ velocity: { spin: [1, 0, 0] } }]),
  ).toThrow("Unknown rigid body velocity option: spin");
  expect(() =>
    Reflect.construct(RigidBody, [{ velocity: { linear: [1, 2] } }]),
  ).toThrow("three components");
});
it("forgets the world of a body its scene no longer holds", async () => {
  const scene = new Scene();
  const ball = body("ball");
  scene.add(ball);
  const world = await create(scene);
  expect(ball.world).toBe(world);
  scene.remove(ball);
  expect(ball.world).toBeUndefined();
  expect(() => {
    if (ball.world) ball.getVelocity();
  }).not.toThrow();
  expect(() => ball.getVelocity()).toThrow("not under a built world's scene");
});
it("undoes a failed join so objects join afresh from their options", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball", { velocity: { linear: [2, 0, 0] } });
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

it("rejects commands on a removed body and restarts it from its options as it rejoins", async () => {
  const scene = new Scene();
  const ball = body("ball", { velocity: { linear: [1, 0, 0] } });
  scene.add(ball);
  const world = await create(scene);
  ball.setVelocity({ linear: new Vector3(3, 0, 0) });
  world.events.length = 0;

  ball.removeFromParent();
  expect(() => ball.teleport(new Matrix4())).toThrow(
    "ball is not under a built world's scene",
  );
  world.update(0);
  expect(world.events).toEqual(["remove ball"]);
  expect(ball.world).toBeUndefined();

  scene.add(ball);
  expect(ball.getVelocity().linear.x).toBe(1);
});
it("clones a body waiting under the scene without joining it", async () => {
  const scene = new Scene();
  const world = await create(scene);
  const ball = body("ball", { velocity: { linear: [2, 0, 0] } });
  scene.add(ball);
  const copy = ball.clone();
  expect(copy.options).toBe(ball.options);
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

it("rejects commands until the world is built and then joins on demand", async () => {
  const scene = new Scene();
  const ball = body("ball");
  scene.add(ball);
  const building = create(scene);
  expect(() => ball.getVelocity()).toThrow("not under a built world's scene");
  expect(ball.world).toBeUndefined();
  const world = await building;
  expect(ball.world).toBe(world);
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
