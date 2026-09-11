import { beforeEach, afterEach } from "vitest";
import { AuthoringWorld, setDefaultWorld } from "../src/index.js";
let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
  setDefaultWorld(world);
});
afterEach(() => world.dispose());
import { describe, expect, it } from "vitest";
import {
  BoxGeometry,
  BufferGeometry,
  CapsuleGeometry,
  Float32BufferAttribute,
  Group,
  Matrix4,
  Vector3,
  Mesh,
  SphereGeometry,
} from "three";
import {
  BoxCollider,
  DistanceJoint,
  RevoluteJoint,
  RigidBody,
} from "../src/index.js";

function doorAssembly() {
  const root = new Group();
  const frame = new RigidBody({ type: "static" });
  for (const x of [-0.55, 0.55]) {
    const post = new Mesh(new BoxGeometry(0.1, 2.2, 0.15));
    post.position.set(x, 1.1, 0);
    frame.add(post);
  }
  const top = new Mesh(new BoxGeometry(1.2, 0.1, 0.15));
  top.position.y = 2.25;
  frame.add(top);
  const door = new RigidBody({ mass: 20 });
  door.position.set(0, 1.05, 0);
  door.add(new Mesh(new BoxGeometry(0.98, 2, 0.06)));
  const hinge = new RevoluteJoint({
    body0: frame,
    body1: door,
    limits: [0, Math.PI / 2],
  });
  hinge.position.set(-0.49, 1.05, 0);
  root.add(frame, door, hinge);
  return { root, frame, door, hinge };
}
describe("physics objects", () => {
  it("preserves the frame opening and resolves hinge frames under transformed parents", () => {
    const { root, frame, door, hinge } = doorAssembly();
    root.position.set(3, 4, 5);
    root.rotation.y = 0.7;
    const colliders = [frame.getColliders(), door.getColliders()];
    expect(colliders.map((shapes) => shapes.length)).toEqual([3, 1]);
    expect(
      colliders.flatMap((shapes) =>
        shapes.map((collider) => collider.shape().kind),
      ),
    ).toEqual(["box", "box", "box", "box"]);
    const frame0 = new Vector3().setFromMatrixPosition(
      hinge.getFrame(0, new Matrix4()),
    );
    const frame1 = new Vector3().setFromMatrixPosition(
      hinge.getFrame(1, new Matrix4()),
    );
    expect(frame0.x).toBeCloseTo(-0.49);
    expect(frame0.y).toBeCloseTo(1.05);
    expect(frame1.x).toBeCloseTo(-0.49);
    expect(frame1.y).toBeCloseTo(0);
  });
  it("does not recognize mutated primitive geometry as an unchanged box", () => {
    const body = new RigidBody();
    body.add(new Mesh(new BoxGeometry().translate(1, 0, 0)));
    expect(body.getColliders()[0]?.shape().kind).toBe("mesh");
  });
  it("accepts scale and rejects nested bodies", () => {
    const { root, door } = doorAssembly();
    root.scale.setScalar(2);
    expect(door.getColliders()).toHaveLength(1);
    root.scale.setScalar(1);
    door.add(new BoxCollider());
    expect(door.getColliders()).toEqual([door.children[1]]);
    door.clear();
    door.add(new RigidBody());
    expect(() => door.getColliders()).toThrow();
  });
  it("rejects sheared automatic collider transforms before decomposition", () => {
    const body = new RigidBody();
    const mesh = new Mesh(new BoxGeometry());
    mesh.matrixAutoUpdate = false;
    mesh.matrix.makeShear(0.5, 0, 0, 0, 0, 0);
    body.add(mesh);
    expect(() => body.getColliders()).toThrow();
  });
  it("resolves explicit collider offsets and material overrides", () => {
    const material = { density: 42 };
    const body = new RigidBody({ colliders: false });
    const collider = new BoxCollider({
      size: [1, 2, 3],
      material,
      sensor: true,
    });
    collider.position.y = 3;
    body.add(collider);
    expect(body.getColliders()[0]).toBe(collider);
    expect(body.getColliders()[0]).toMatchObject({
      material,
      sensor: true,
      position: { y: 3 },
    });
  });
  it("rejects incomplete frames", () => {
    const { hinge } = doorAssembly();
    hinge.options.frame0 = new Matrix4();
    expect(() => hinge.validate()).toThrow("both local frames");
  });
  it("supports explicit separated anchors and validates distance limits", () => {
    const body = new RigidBody();
    body.add(new Mesh(new SphereGeometry()));
    const joint = new DistanceJoint({
      body0: null,
      body1: body,
      frame0: new Matrix4().makeTranslation(0, 5, 0),
      frame1: new Matrix4().makeTranslation(0, 1, 0),
      limits: [0, 4],
    });
    const root = new Group();
    root.add(body, joint);
    expect(
      new Vector3().setFromMatrixPosition(joint.getFrame(0, new Matrix4())).y,
    ).toBe(5);
    joint.options.limits = [-1, 4];
    expect(() => joint.validate()).toThrow("nonnegative");
  });
  it("rejects massless dynamic bodies and invalid collision masks", () => {
    const body = new RigidBody({
      colliders: false,
      material: { density: 0 },
    });
    const collider = new BoxCollider();
    body.add(collider);
    expect(() => body.getColliders()).toThrow("positive mass");
    body.options.mass = 1;
    collider.collisionGroups = { membership: 65536, filter: 1 };
    expect(() => body.getColliders()).toThrow("16-bit");
    collider.collisionGroups = { membership: 2, filter: 4 };
    expect(body.getColliders()[0]?.collisionGroups).toEqual({
      membership: 2,
      filter: 4,
    });
  });
  it("rejects malformed triangle meshes before passing them to an adapter", () => {
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0], 3),
    );
    geometry.setIndex([0, 1, 3]);
    const body = new RigidBody({ type: "static" });
    body.add(new Mesh(geometry));
    expect(() => body.getColliders()).toThrow("index is outside");
    geometry.setIndex([0, 1]);
    expect(() => body.getColliders()).toThrow("complete triangles");
  });
  it("matches sphere and capsule extents to the visual primitives", () => {
    const capsule = new CapsuleGeometry(0.25, 1.5);
    capsule.computeBoundingBox();
    const body = new RigidBody();
    body.add(new Mesh(capsule), new Mesh(new SphereGeometry(0.75)));
    const shapes = body.getColliders().map((collider) => collider.shape());
    expect(shapes).toEqual([
      { kind: "capsule", radius: 0.25, length: 1.5 },
      { kind: "sphere", radius: 0.75 },
    ]);
    expect(capsule.boundingBox?.min.y).toBeCloseTo(-1);
    expect(capsule.boundingBox?.max.y).toBeCloseTo(1);
  });
  it("rejects partial draw ranges rather than colliding with invisible triangles", () => {
    const geometry = new BoxGeometry();
    geometry.setDrawRange(0, 3);
    const body = new RigidBody();
    body.add(new Mesh(geometry));
    expect(() => body.getColliders()).toThrow("full draw range");
  });
});

it("uses explicit colliders before inspecting visual geometry and regenerates when removed", () => {
  const body = new RigidBody();
  const mesh = new Mesh(new BoxGeometry());
  mesh.scale.setScalar(2);
  const collider = new BoxCollider();
  body.add(mesh, new Group().add(collider));
  expect(body.getColliders()).toEqual([collider]);
  collider.removeFromParent();
  expect(body.getColliders()[0]).toBeInstanceOf(BoxCollider);
  mesh.scale.setScalar(1);
  expect(body.getColliders()[0]).toBeInstanceOf(BoxCollider);
});

it("resolves plain material options without mutating them", () => {
  const material = { density: 42 };
  const body = new RigidBody({ material });
  const collider = new BoxCollider();
  body.add(collider);
  expect(body.getMaterial(collider)).toEqual({
    density: 42,
    staticFriction: 0.5,
    dynamicFriction: 0.5,
    restitution: 0,
  });
  expect(material).toEqual({ density: 42 });
  material.density = 43;
  expect(body.getMaterial(collider).density).toBe(43);
  collider.material = { restitution: 0.9 };
  expect(body.getMaterial(collider).restitution).toBe(0.9);
});
