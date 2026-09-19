import { afterEach, expect, it } from "vitest";
import { BoxGeometry, Group, Matrix4, Mesh, Vector3 } from "three";
import {
  registry,
  RigidBody,
  BoxCollider,
  SphereCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
  FixedJoint,
  resolveCollider,
} from "../src/index.js";
afterEach(() => registry.clear());

it("combines ancestor, body and collider scale including offsets without mutating sources", () => {
  const root = new Group();
  root.scale.setScalar(2);
  root.rotation.y = 0.4;
  const body = new RigidBody();
  body.scale.set(1, 2, 3);
  const collider = new BoxCollider({ size: [2, 2, 2] });
  collider.scale.set(3, 2, 1);
  collider.position.set(1, 2, 3);
  root.add(body.add(collider));
  body.getColliders();
  const resolved = resolveCollider(body, collider);
  expect(resolved.shape).toEqual({ kind: "box", size: [12, 16, 12] });
  expect(
    new Vector3()
      .setFromMatrixPosition(resolved.matrix)
      .distanceTo(new Vector3(2, 8, 18)),
  ).toBeLessThan(1e-6);
  expect(collider.size).toEqual([2, 2, 2]);
});

it("scales primitive dimensions and mesh vertices", () => {
  const body = new RigidBody();
  body.scale.setScalar(2);
  const geometry = new BoxGeometry();
  const original = Array.from(geometry.getAttribute("position").array);
  const colliders = [
    new SphereCollider(),
    new CapsuleCollider(),
    new CylinderCollider(),
    new MeshCollider().setGeometry(geometry),
  ];
  body.add(...colliders);
  body.getColliders();
  const shapes = colliders.map(
    (collider) => resolveCollider(body, collider).shape,
  );
  expect(shapes.slice(0, 3)).toEqual([
    { kind: "sphere", radius: 1 },
    { kind: "capsule", radius: 1, height: 2 },
    { kind: "cylinder", radius: 1, height: 2 },
  ]);
  const mesh = shapes[3];
  if (mesh?.kind !== "mesh") throw new Error("Missing mesh");
  expect(Array.from(mesh.geometry.getAttribute("position").array)).toEqual(
    original.map((value) => value * 2),
  );
  expect(Array.from(geometry.getAttribute("position").array)).toEqual(original);
});

it("allows independent cylinder height but rejects stretched round primitives", () => {
  const body = new RigidBody();
  body.scale.set(2, 3, 2);
  const cylinder = new CylinderCollider();
  body.add(cylinder);
  body.getColliders();
  expect(resolveCollider(body, cylinder).shape).toEqual({
    kind: "cylinder",
    radius: 1,
    height: 3,
  });
  for (const collider of [new SphereCollider(), new CapsuleCollider()]) {
    body.add(collider);
    body.getColliders();
    expect(() => resolveCollider(body, collider)).toThrow("nonuniform");
  }
});

it("scales inferred and explicit joint anchors into rigid body coordinates", () => {
  const root = new Group();
  root.scale.setScalar(3);
  const body = new RigidBody();
  body.position.y = 2;
  body.scale.setScalar(2);
  body.add(new Mesh(new BoxGeometry()));
  const inferred = new FixedJoint({ body0: null, body1: body });
  inferred.position.y = 3;
  root.add(body, inferred);
  expect(
    new Vector3().setFromMatrixPosition(inferred.getFrame(1, new Matrix4())).y,
  ).toBe(3);
  const explicit = new FixedJoint({
    body0: null,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4().makeTranslation(0, 1, 0),
  });
  expect(
    new Vector3().setFromMatrixPosition(explicit.getFrame(1, new Matrix4())).y,
  ).toBe(6);
});

it("rejects singular transforms, shear and nonuniform moving ancestors", () => {
  const root = new Group();
  const body = new RigidBody();
  root.add(body.add(new Mesh(new BoxGeometry())));
  for (const x of [0, -1, NaN]) {
    body.scale.x = x;
    expect(() => body.getColliders()).toThrow("positive scale");
  }
  body.scale.set(-1, -1, 1);
  expect(() => body.getColliders()).toThrow("positive scale");
  body.scale.setScalar(1);
  root.scale.set(2, 1, 1);
  expect(() => body.getColliders()).toThrow("uniform ancestor");
  const fixed = new RigidBody({ type: "static" });
  root.add(fixed.add(new Mesh(new BoxGeometry())));
  fixed.rotation.z = 0.5;
  expect(() => fixed.getColliders()).toThrow("shear");
});

it("rejects world shear before decomposing automatic or explicit colliders", () => {
  for (const source of [new Mesh(new BoxGeometry()), new BoxCollider()]) {
    const body = new RigidBody();
    body.scale.set(2, 1, 1);
    source.rotation.z = 0.5;
    body.add(source);
    expect(() => body.getColliders()).toThrow("shear");
  }
});
