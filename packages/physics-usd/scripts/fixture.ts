import { writeFileSync } from "node:fs";
import { Scene, Group, Mesh, MeshStandardMaterial, BoxGeometry } from "three";
import {
  AuthoringWorld,
  setDefaultWorld,
  RigidBody,
  RevoluteJoint,
  JointMotor,
} from "@drawcall/physics";
import { PhysicsUSDExporter } from "../dist/index.js";
const world = new AuthoringWorld();
setDefaultWorld(world);
const scene = new Scene();
const assembly = new Group();
assembly.position.set(2, 3, 4);
assembly.scale.setScalar(Number(process.argv[3] ?? 1));
assembly.rotation.y = Math.PI / 4;
const frame = new RigidBody().setType("static");
frame.name = "Frame";
const door = new RigidBody({ mass: 20 });
door.name = "Door";
door.position.set(0.5, 1, 0);
const material = new MeshStandardMaterial();
frame.add(new Mesh(new BoxGeometry(0.1, 2, 0.1), material));
door.add(new Mesh(new BoxGeometry(1, 2, 0.06), material));
const hinge = new RevoluteJoint({
  body0: frame,
  body1: door,
  limits: [0, Math.PI / 2],
});
new JointMotor({
  joint: hinge,
  stiffness: 100,
  damping: 10,
  maxForce: 40,
}).setTarget({ position: Math.PI / 4, velocity: 0 });
hinge.position.set(0, 1, 0);
assembly.add(frame, door, hinge);
scene.add(assembly);
writeFileSync(
  process.argv[2] ?? "/tmp/door.usdz",
  await new PhysicsUSDExporter().parseAsync(scene),
);

world.dispose();
