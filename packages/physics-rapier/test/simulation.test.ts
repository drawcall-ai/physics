import { describe, expect, it } from "vitest";
import { BoxGeometry, Group, Matrix4, Mesh, Vector3 } from "three";
import {
  BoxCollider,
  JointMotor,
  DistanceJoint,
  FixedJoint,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
  SphericalJoint,
} from "@drawcall/physics";
import { setupWorld, type RapierWorld } from "../src/index.js";

function box(type: "dynamic" | "static" | "kinematic" = "dynamic") {
  const body = new RigidBody({ mass: 1 }).setType(type);
  body.add(new Mesh(new BoxGeometry(1, 1, 1)));
  return body;
}
function steps(simulation: RapierWorld, count = 120) {
  for (let i = 0; i < count; i++) simulation.update(simulation.fixedDelta);
}

describe("RapierWorld", () => {
  it("updates manual body matrices when synchronizing simulated poses", async () => {
    const simulation = await setupWorld();
    const scene = new Group(),
      body = box();
    body.position.y = 2;
    body.updateMatrix();
    body.matrixAutoUpdate = false;
    scene.add(body);

    simulation.update(simulation.fixedDelta);
    expect(body.position.y).toBeLessThan(2);
    expect(body.getWorldPosition(new Vector3()).y).toBeCloseTo(
      body.position.y,
      6,
    );
    simulation.dispose();
  });
  it("rejects invalid effort commands before stepping", async () => {
    const simulation = await setupWorld();
    const scene = new Group(),
      body = box();
    const hinge = new RevoluteJoint({ body0: null, body1: body });
    scene.add(body, hinge);

    expect(() => hinge.setEffort(NaN)).toThrow("finite");
    expect(() => hinge.setEffort(Infinity)).toThrow("finite");
    simulation.dispose();
  });
  it("distributes explicit mass over compound colliders", async () => {
    const simulation = await setupWorld({
      gravity: [0, 0, 0],
    });
    const scene = new Group(),
      body = new RigidBody({ mass: 20 });
    const a = new Mesh(new BoxGeometry(1, 1, 1));
    a.position.x = -1;
    const b = new Mesh(new BoxGeometry(2, 1, 1));
    b.position.x = 1;
    body.add(a, b);
    scene.add(body);

    simulation.update(simulation.fixedDelta);
    body.applyImpulse(new Vector3(10, 0, 0));
    simulation.update(simulation.fixedDelta);
    expect(body.getVelocity().linear.x).toBeCloseTo(0.5, 5);
    body.applyImpulse(new Vector3(0, 1, 0), new Vector3(3, 0, 0));
    simulation.update(simulation.fixedDelta);
    expect(body.getVelocity().angular.z).toBeGreaterThan(0);
    simulation.dispose();
  });
  it("honors collision membership and filter masks", async () => {
    const simulation = await setupWorld();
    const scene = new Group();
    const floor = new RigidBody({ colliders: false }).setType("static");
    floor.add(
      new BoxCollider({ size: [10, 1, 10] }).setCollisionGroups({
        membership: 1,
        filter: 1,
      }),
    );
    const falling = new RigidBody({ colliders: false, mass: 1 });
    falling.add(
      new BoxCollider().setCollisionGroups({ membership: 2, filter: 2 }),
    );
    falling.position.y = 2;
    scene.add(floor, falling);

    steps(simulation, 60);
    expect(falling.position.y).toBeLessThan(-1);
    simulation.dispose();
  });
  it("keeps the opening of a compound frame empty", async () => {
    const simulation = await setupWorld();
    const scene = new Group(),
      frame = new RigidBody({}).setType("static"),
      falling = box();
    for (const x of [-2, 2]) {
      const post = new Mesh(new BoxGeometry(0.2, 4, 0.2));
      post.position.set(x, 2, 0);
      frame.add(post);
    }
    const lintel = new Mesh(new BoxGeometry(4.2, 0.2, 0.2));
    lintel.position.y = 4;
    frame.add(lintel);
    falling.position.y = 2;
    scene.add(frame, falling);

    steps(simulation, 60);
    expect(falling.position.y).toBeLessThan(0);
    simulation.dispose();
  });
  it("releases disposed bodies and rejects further operations", async () => {
    const simulation = await setupWorld();
    const body = box();
    body.dispose();
    simulation.update(simulation.fixedDelta);
    expect(() => body.applyImpulse(new Vector3(1, 0, 0))).toThrow("disposed");
    simulation.dispose();
  });
  it("drops a body onto a floor, resets its pose, and rejects disposed access", async () => {
    const simulation = await setupWorld();
    const scene = new Group();
    const floor = box("static"),
      falling = box();
    floor.position.y = -0.5;
    falling.position.y = 3;
    scene.add(floor, falling);

    steps(simulation);
    expect(falling.position.y).toBeCloseTo(0.5, 1);
    simulation.reset();
    expect(falling.position.y).toBe(3);
    simulation.dispose();
    expect(() => falling.getVelocity()).toThrow("disposed");
  });
  it("drives a hinge under rotated parents while preserving its anchor", async () => {
    const simulation = await setupWorld({
      gravity: [0, 0, 0],
    });
    const scene = new Group(),
      assembly = new Group();
    assembly.position.set(5, 2, 3);
    assembly.rotation.z = 0.4;
    scene.add(assembly);
    const frame = box("static"),
      door = box();
    door.position.x = 1;
    const hinge = new RevoluteJoint({
      body0: frame,
      body1: door,
      limits: [0, 1.5],
    });

    new JointMotor({ joint: hinge, stiffness: 50, damping: 10 }).setTarget({
      position: 1,
    });
    hinge.position.x = 0.5;
    assembly.add(frame, door, hinge);

    steps(simulation, 240);
    expect(hinge.getState().position).toBeCloseTo(1, 1);
    const frame0 = hinge
      .getFrame(0, new Matrix4())
      .premultiply(frame.matrixWorld);
    const frame1 = hinge
      .getFrame(1, new Matrix4())
      .premultiply(door.matrixWorld);
    expect(
      new Vector3()
        .setFromMatrixPosition(frame0)
        .distanceTo(new Vector3().setFromMatrixPosition(frame1)),
    ).toBeLessThan(0.01);
    simulation.dispose();
  });
  it("drives a slider and surfaces invalid live material edits", async () => {
    const simulation = await setupWorld({
      gravity: [0, 0, 0],
    });
    const scene = new Group(),
      body = box();
    scene.add(body);
    const slider = new PrismaticJoint({
      body0: null,
      body1: body,
      axis: "X",
      limits: [0, 2],
    });

    new JointMotor({ joint: slider, stiffness: 50, damping: 10 }).setTarget({
      position: 1,
    });
    scene.add(slider);

    steps(simulation);
    expect(body.position.x).toBeCloseTo(1, 1);
    body.setMaterial({ staticFriction: 1, dynamicFriction: 0.2 });
    expect(() => simulation.update(simulation.fixedDelta)).toThrow("friction");
    body.setMaterial({});
    simulation.update(simulation.fixedDelta);
    expect(new Vector3().setFromMatrixPosition(body.matrixWorld).x).toBeCloseTo(
      1,
      1,
    );
    simulation.dispose();
  });
  it("supports fixed, spherical, and rope constraints", async () => {
    for (const kind of ["fixed", "spherical", "distance"]) {
      const simulation = await setupWorld();
      const scene = new Group(),
        body = box();
      body.position.y = -2;
      scene.add(body);
      const joint =
        kind === "fixed"
          ? new FixedJoint({ body0: null, body1: body })
          : kind === "spherical"
            ? new SphericalJoint({ body0: null, body1: body })
            : new DistanceJoint({
                body0: null,
                body1: body,
                frame0: new Matrix4(),
                frame1: new Matrix4(),
                limits: [0, 2],
              });
      scene.add(joint);

      steps(simulation);
      expect(body.position.y).toBeCloseTo(-2, 1);
      simulation.dispose();
    }
  });
  it("applies forces for one step and accepts kinematic targets", async () => {
    const simulation = await setupWorld({
      gravity: [0, 0, 0],
    });
    const scene = new Group(),
      body = box(),
      kinematic = box("kinematic");
    kinematic.position.x = 5;
    scene.add(body, kinematic);

    simulation.update(simulation.fixedDelta);
    body.applyForce(new Vector3(60, 0, 0));
    simulation.update(simulation.fixedDelta);
    const velocity = body.getVelocity().linear.x;
    simulation.update(simulation.fixedDelta);
    expect(body.getVelocity().linear.x).toBeCloseTo(velocity);
    kinematic.setKinematicTarget(new Matrix4().makeTranslation(6, 0, 0));
    simulation.update(simulation.fixedDelta);
    expect(kinematic.position.x).toBeCloseTo(6);
    simulation.dispose();
  });
});
