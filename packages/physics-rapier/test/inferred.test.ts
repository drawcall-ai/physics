import { expect, it } from "vitest";
import {
  BufferGeometry,
  Float32BufferAttribute,
  BoxGeometry,
  Group,
  Matrix4,
  Mesh,
  Quaternion,
  Vector3,
} from "three";
import { BoxCollider, PrismaticJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "../src/index.js";

it("preserves anchor velocity across preparation for a density-weighted compound body", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const body = new RigidBody({ mass: 10 }).setVelocity({
    angular: new Vector3(0, 0, 2),
  });
  const light = new BoxCollider().setMaterial({ density: 1 });
  const heavy = new BoxCollider().setMaterial({ density: 9 });
  heavy.position.x = 4;
  body.add(light, heavy);
  const slider = new PrismaticJoint({
    body0: null,
    body1: body,
    axis: "Y",
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  }).setEnabled(false);

  const authored = slider.getState();
  expect(authored.velocity).toBeCloseTo(-7.2, 5);
  expect(world.time).toBe(0);
  expect(
    world.raycast(new Vector3(-10, 0, 0), new Vector3(1, 0, 0), 20),
  ).toBeNull();
  world.update(0);
  expect(slider.getState().velocity).toBeCloseTo(authored.velocity, 5);
  expect(slider.getState().position).toBeCloseTo(authored.position, 5);
  world.dispose();
});

it("infers a translated convex hull's volume centroid before preparation", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const body = new RigidBody({ colliders: "convexHull" }).setVelocity({
    angular: new Vector3(0, 0, 2),
  });
  const mesh = new Mesh(new BoxGeometry(2, 1, 1).translate(2, 0, 0));
  mesh.position.x = 1;
  body.add(mesh);
  const slider = new PrismaticJoint({
    body0: null,
    body1: body,
    axis: "Y",
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  }).setEnabled(false);

  const authored = slider.getState();
  expect(authored.velocity).toBeCloseTo(-6, 5);
  world.update(0);
  expect(slider.getState().velocity).toBeCloseTo(authored.velocity, 5);
  world.dispose();
});

it("uses the solid hull centroid independently of interior vertex sampling", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const geometry = new BufferGeometry().setAttribute(
    "position",
    new Float32BufferAttribute(
      [
        1000, 0, 0, 1004, 0, 0, 1000, 2, 0, 1000, 0, 1, 1000.25, 0.25, 0.25,
        1000.5, 0.25, 0.25,
      ],
      3,
    ),
  );
  const body = new RigidBody({ colliders: "convexHull" }).setVelocity({
    angular: new Vector3(0, 0, 2),
  });
  body.add(new Mesh(geometry));
  const slider = new PrismaticJoint({
    body0: null,
    body1: body,
    axis: "Y",
    frame0: new Matrix4(),
    frame1: new Matrix4(),
  }).setEnabled(false);
  const authored = slider.getState();
  expect(authored.velocity).toBeCloseTo(-2002, 4);
  world.update(0);
  expect(slider.getState().velocity).toBeCloseTo(authored.velocity, 3);
  world.dispose();
});

it("infers centers through rotated and uniformly scaled parents without freezing assembly", async () => {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  const parent = new Group();
  parent.position.set(10, 20, 30);
  parent.rotation.set(0.2, 0.3, 0.4);
  parent.scale.setScalar(2);
  parent.updateMatrixWorld(true);
  const rotation = parent.getWorldQuaternion(new Quaternion());
  const body = new RigidBody({ mass: 10 }).setVelocity({
    angular: new Vector3(0, 0, 3).applyQuaternion(rotation),
  });
  const light = new BoxCollider().setMaterial({ density: 1 });
  const heavy = new BoxCollider().setMaterial({ density: 9 });
  heavy.position.x = 4;
  body.add(light, heavy);
  parent.add(body);
  const slider = new PrismaticJoint({
    body0: null,
    body1: body,
    axis: "Y",
    frame0: new Matrix4().compose(
      parent.position,
      rotation,
      new Vector3(1, 1, 1),
    ),
    frame1: new Matrix4(),
  }).setEnabled(false);

  expect(slider.getState().velocity).toBeCloseTo(-21.6, 5);
  body.scale.setScalar(1.5);
  const authored = slider.getState();
  expect(authored.velocity).toBeCloseTo(-32.4, 5);
  world.update(0);
  expect(slider.getState().velocity).toBeCloseTo(authored.velocity, 4);
  world.dispose();
});
