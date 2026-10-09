// A driven six-joint arm above a table of mesh-collider pieces, stepped at 500 Hz: a scene where
// almost nothing changes between steps but drive targets and the bodies physics moves.
// Usage, after building: pnpm bench:arm [path to a built index.js] [pieces] [steps]
// Prints the wall time per step and a checksum of the final body positions as JSON.
import {
  BoxCollider,
  JointDrive,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { CylinderGeometry, Matrix4, Mesh, Scene } from "three";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";

const [entry = "dist/index.js", count = "16", length = "3000"] =
  process.argv.slice(2);
const { buildWorld } = (await import(
  pathToFileURL(resolve(entry)).href
)) as typeof import("../src/index.js");
const pieces = Number(count);
const steps = Number(length);

// Hulls like a robot link's or a chess piece's, and a visual mesh beside each.
const scene = new Scene();
const table = new RigidBody({ bodyType: "static" });
table.add(new BoxCollider({ size: [2, 0.1, 2] }));
table.position.y = -0.05;
const base = new RigidBody({ bodyType: "static" });
base.add(new BoxCollider({ size: [0.1, 0.05, 0.1] }));
base.position.set(0, 0.2, -0.5);
scene.add(table, base);
const drives: JointDrive[] = [];
const bodies: RigidBody[] = [];
let parent = base;
for (let i = 0; i < 6; i++) {
  const link = new RigidBody({ mass: 0.1, canSleep: false });
  bodies.push(link);
  scene.add(link);
  link.position.set(0, 0.2 + 0.06 * (i + 1), -0.5);
  const geometry = new CylinderGeometry(0.02, 0.02, 0.05, 24);
  link.add(
    new Mesh(geometry),
    new MeshCollider().setGeometry(geometry.toNonIndexed()),
  );
  link.add(new BoxCollider({ size: [0.03, 0.01, 0.03] }));
  const drive = new JointDrive({ stiffness: 500, damping: 2, maxForce: 3 });
  scene.add(
    new RevoluteJoint({
      body0: parent,
      body1: link,
      frame0: new Matrix4().makeTranslation(0, 0.06, 0),
      frame1: new Matrix4(),
      axis: i % 2 ? "X" : "Y",
    }).setDrive(drive),
  );
  drives.push(drive);
  parent = link;
}
for (let i = 0; i < pieces; i++) {
  const body = new RigidBody();
  bodies.push(body);
  scene.add(body);
  body.position.set((i % 4) * 0.1 - 0.15, 0.03, Math.floor(i / 4) * 0.1 - 0.15);
  const geometry = new CylinderGeometry(0.015, 0.02, 0.05, 16);
  const collider = new MeshCollider().setGeometry(geometry.toNonIndexed());
  collider.scale.set(1.1, 1, 1.1);
  body.add(new Mesh(geometry), collider);
  body.add(new BoxCollider({ size: [0.01, 0.005, 0.01] }));
}

const world = await buildWorld({ scene, fixedDelta: 1 / 500 });
let step = 0;
world.onBeforeStep(() => {
  step++;
  drives.forEach((drive, i) =>
    drive.setTarget({ position: 0.3 * Math.sin(step * 0.002 + i) }),
  );
});
world.update(world.fixedDelta);
const start = performance.now();
for (let i = 0; i < steps; i++) world.update(world.fixedDelta);
const elapsed = performance.now() - start;
// Equal across builds when the simulation is unchanged.
let checksum = 0;
for (const body of bodies) checksum += body.position.x + body.position.y;
console.log(
  JSON.stringify({
    pieces,
    steps,
    msPerStep: +(elapsed / steps).toFixed(4),
    checksum: +checksum.toFixed(9),
  }),
);
world.dispose();
