import { expect, test } from "vitest";
import {
  DistanceJoint,
  type PhysicsMaterial,
  JointDrive,
  RevoluteJoint,
  RigidBody,
  SphereCollider,
} from "@drawcall/physics";
import { Vector3 } from "three";
import {
  body,
  createWorld,
  earth,
  ground,
  resetScene,
  scene,
  steps,
} from "./fixtures.js";

test("joint friction holds a lever its load cannot turn and lets a heavier load through", async () => {
  // A 1 kg lever 0.5 m from a horizontal hinge, so gravity turns it with 4.9 N·m.
  const swing = async (frictionLoss: number) => {
    resetScene();
    const lever = new RigidBody({
      mass: 1,
      centerOfMass: [0.5, 0, 0],
      diagonalInertia: [0.01, 0.01, 0.01],
      colliders: false,
    });
    scene.add(lever);
    const hinge = new RevoluteJoint({ body0: null, body1: lever, axis: "Z" });
    hinge.setDrive(new JointDrive({ frictionLoss }));
    scene.add(hinge);
    const world = await createWorld(earth);
    steps(world, 200);
    return hinge.getState().position;
  };
  expect(Math.abs(await swing(6))).toBeLessThan(0.01);
  expect(Math.abs(await swing(3))).toBeGreaterThan(0.5);
});

test("armature adds to the inertia an effort accelerates", async () => {
  const spin = async (armature: number) => {
    resetScene();
    const lever = new RigidBody({
      mass: 1,
      centerOfMass: [0, 0, 0],
      diagonalInertia: [1, 1, 1],
      colliders: false,
    });
    scene.add(lever);
    const hinge = new RevoluteJoint({ body0: null, body1: lever, axis: "Z" });
    hinge.setDrive(new JointDrive({ armature }).setTarget({ effort: 1 }));
    scene.add(hinge);
    const world = await createWorld();
    steps(world, 100);
    return hinge.getState().velocity;
  };
  expect(await spin(0)).toBeCloseTo(1, 6);
  expect(await spin(1)).toBeCloseTo(0.5, 6);
});

/** A 1 kg, 0.1 m radius ball resting on the ground, which touches it at a single point. */
function ball(material: PhysicsMaterial): RigidBody {
  ground();
  const ball = new RigidBody({ mass: 1 });
  ball.add(new SphereCollider({ radius: 0.1 }).setMaterial(material));
  ball.position.y = 0.1;
  scene.add(ball);
  return ball;
}

test("torsional friction stops a ball spinning on the ground", async () => {
  const spin = async (torsionalFriction: number) => {
    resetScene();
    const spinning = ball({ torsionalFriction });
    const world = await createWorld(earth);
    steps(world, 20);
    spinning.setVelocity({ angular: new Vector3(0, 5, 0) });
    steps(world, 100);
    return Math.abs(spinning.getVelocity().angular.y);
  };
  expect(await spin(0)).toBeGreaterThan(4);
  expect(await spin(0.01)).toBeLessThan(0.1);
});

test("rolling friction stops a rolling ball", async () => {
  const roll = async (rollingFriction: number) => {
    resetScene();
    const rolling = ball({ rollingFriction });
    const world = await createWorld(earth);
    steps(world, 20);
    rolling.setVelocity({
      linear: new Vector3(1, 0, 0),
      angular: new Vector3(0, 0, -10),
    });
    steps(world, 200);
    return Math.abs(rolling.getVelocity().linear.x);
  };
  expect(await roll(0)).toBeGreaterThan(0.5);
  expect(await roll(0.02)).toBeLessThan(0.05);
});

test("rejects friction and armature on drives without a joint coordinate", async () => {
  const joint = new DistanceJoint({
    body0: null,
    body1: body(),
    limits: [0, 1],
  });
  joint.setDrive(new JointDrive({ frictionLoss: 1 }));
  scene.add(joint);
  await expect(createWorld()).rejects.toThrow(
    "only on joints with coordinates",
  );
});
