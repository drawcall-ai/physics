// Contact-heavy scene: chess-like pieces, each a concave triangle mesh that MuJoCo collides as
// convex parts, resting on a box board on a box desk, and two driven box pads gripping one.
// Usage, after building: pnpm bench [path to a built index.js] [seconds]
// Prints the wall time of the stepping and every piece's final pose as JSON.
import {
  BoxCollider,
  JointDrive,
  MeshCollider,
  PrismaticJoint,
  RigidBody,
} from "@drawcall/physics";
import { BufferGeometry, Float32BufferAttribute, Matrix4, Scene } from "three";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";

const [entry = "dist/index.js", seconds = "4"] = process.argv.slice(2);
const { buildWorld } = (await import(
  pathToFileURL(resolve(entry)).href
)) as typeof import("../src/index.js");

// A wide flat base, a narrower body and a head: rings of (radius, height) closed by two poles.
const rings = [
  [0.016, 0],
  [0.016, 0.008],
  [0.01, 0.012],
  [0.007, 0.03],
  [0.011, 0.034],
  [0.011, 0.038],
  [0.006, 0.044],
];
const sides = 16;
const vertices = [0, 0, 0, 0, 0.046, 0];
for (const [radius, height] of rings)
  for (let i = 0; i < sides; i++) {
    const angle = (i / sides) * 2 * Math.PI;
    vertices.push(radius * Math.cos(angle), height, -radius * Math.sin(angle));
  }
const ring = (r: number, i: number) => 2 + r * sides + (i % sides);
const faces: number[] = [];
for (let i = 0; i < sides; i++) {
  faces.push(0, ring(0, i + 1), ring(0, i));
  faces.push(1, ring(rings.length - 1, i), ring(rings.length - 1, i + 1));
  for (let r = 0; r + 1 < rings.length; r++)
    faces.push(
      ring(r, i),
      ring(r, i + 1),
      ring(r + 1, i + 1),
      ring(r, i),
      ring(r + 1, i + 1),
      ring(r + 1, i),
    );
}
const piece = new BufferGeometry()
  .setAttribute("position", new Float32BufferAttribute(vertices, 3))
  .setIndex(faces);
const material = { staticFriction: 0.8, dynamicFriction: 0.8 };

const scene = new Scene();
const desk = new RigidBody({ type: "static" });
desk.add(new BoxCollider({ size: [1, 0.7, 1] }));
desk.position.y = 0.35;
const board = new RigidBody({ type: "static" });
board.add(new BoxCollider({ size: [0.42, 0.02, 0.42] }));
board.position.y = 0.71;
scene.add(desk, board);
const top = 0.72;
const pieces: RigidBody[] = [];
for (let i = 0; i < 32; i++) {
  const body = new RigidBody({ mass: 0.02 }).setMaterial(material);
  body.add(new MeshCollider({ approximation: "trimesh" }).setGeometry(piece));
  body.position.set(
    -0.175 + (i % 8) * 0.05,
    top + 0.0005,
    i < 16
      ? -0.175 + Math.floor(i / 8) * 0.05
      : 0.125 + Math.floor((i - 16) / 8) * 0.05,
  );
  pieces.push(body);
  scene.add(body);
}
const held = pieces[0]!;
for (const side of [-1, 1]) {
  const pad = new RigidBody({ mass: 0.05 }).setMaterial(material);
  pad.add(new BoxCollider({ size: [0.01, 0.01, 0.03] }));
  const x = held.position.x + side * 0.02;
  pad.position.set(x, top + 0.02, held.position.z);
  scene.add(pad);
  new PrismaticJoint({
    body0: null,
    body1: pad,
    axis: "X",
    frame0: new Matrix4().makeTranslation(x, top + 0.02, held.position.z),
    frame1: new Matrix4(),
  }).setDrive(
    // Squeeze past the piece so the force limit sets the grip.
    new JointDrive({ stiffness: 200, damping: 5, maxForce: 2 }).setTarget({
      position: -side * 0.02,
    }),
  );
}

const world = await buildWorld(scene, {
  fixedDelta: 1 / 500,
  solverIterations: 50,
  frictionCone: "elliptic",
  frictionImpedanceRatio: 50,
});
const steps = Math.round(Number(seconds) / world.fixedDelta);
world.update(0);
const start = performance.now();
for (let i = 0; i < steps; i++) world.update(world.fixedDelta);
const ms = performance.now() - start;
console.log(
  JSON.stringify({
    ms,
    stepMs: ms / steps,
    poses: pieces.map((body) => [
      ...body.position.toArray(),
      ...body.quaternion.toArray(),
    ]),
  }),
);
world.dispose();
