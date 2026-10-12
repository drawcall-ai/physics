import { expect, it } from "vitest";
import { BoxGeometry, Group, Matrix4, Mesh, Scene, Vector3 } from "three";
import {
  JointDrive,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
  SphericalJoint,
} from "@drawcall/physics";
import { createWorld, box, concave, earth, steps } from "./fixtures.js";

it("keeps the opening of a compound frame empty", async () => {
  const world = await createWorld(earth);
  const scene = world.scene,
    frame = new RigidBody({ bodyType: "static" }),
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
  steps(world, 60);
  expect(falling.position.y).toBeLessThan(0);
});

it("drops a body onto a floor, resets its pose, and stops simulating it once the world is disposed", async () => {
  const world = await createWorld(earth);
  const scene = world.scene;
  const floor = box("static"),
    falling = box();
  floor.position.y = -0.5;
  falling.position.y = 3;
  scene.add(floor, falling);
  steps(world);
  expect(falling.position.y).toBeCloseTo(0.5, 1);
  world.reset();
  expect(falling.position.y).toBe(3);
  world.dispose();
  expect(falling.parent).toBe(scene);
  expect(() => falling.applyImpulse(new Vector3(1, 0, 0))).toThrow(
    "not under a built world's scene",
  );
});

it("drives a hinge under rotated parents while preserving its anchor", async () => {
  const world = await createWorld({ fixedDelta: 1 / 60 });
  const scene = world.scene,
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
  hinge.setDrive(
    new JointDrive({ stiffness: 50, damping: 10 }).setTarget({
      position: 1,
    }),
  );
  hinge.position.x = 0.5;
  assembly.add(frame, door, hinge);
  steps(world, 240);
  expect(hinge.getState().position).toBeCloseTo(1, 1);
  const frame0 = hinge
    .getFrame(0, new Matrix4())
    .premultiply(frame.matrixWorld);
  const frame1 = hinge.getFrame(1, new Matrix4()).premultiply(door.matrixWorld);
  expect(
    new Vector3()
      .setFromMatrixPosition(frame0)
      .distanceTo(new Vector3().setFromMatrixPosition(frame1)),
  ).toBeLessThan(0.01);
});

it("holds a body with a spherical constraint", async () => {
  const world = await createWorld(earth);
  const body = box();
  body.position.y = -2;
  world.scene.add(body, new SphericalJoint({ body0: null, body1: body }));
  steps(world);
  expect(body.position.y).toBeCloseTo(-2, 1);
});

it("collides a triangle mesh on a moving body as its convex parts", async () => {
  const body = new RigidBody();
  body.add(
    new MeshCollider({ approximation: "trimesh" }).setGeometry(concave()),
  );
  const world = await createWorld({ ...earth, scene: new Scene().add(body) });
  const forward = new Vector3(0, 0, -1);
  expect(world.raycast(new Vector3(0.7, 0.7, 3), forward, 6)).toBeNull();
  expect(world.raycast(new Vector3(0.2, 0.7, 3), forward, 6)).toMatchObject({
    kind: "body",
    body,
  });
  for (let i = 0; i < 30; i++) world.update(1 / 60);
  expect(body.position.y).toBeLessThan(-0.5);
});
