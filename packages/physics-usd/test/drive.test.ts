import { expect, it } from "vitest";
import { Group } from "three";
import {
  GenericJoint,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

it("rejects untargeted, effort-driven and speed-limited export instead of changing actuation", async () => {
  const body = new RigidBody({ bodyType: "static" });
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const drive = new JointDrive({ stiffness: 10 });
  joint.setDrive(drive);
  const scene = new Group().add(body, joint);
  const exporter = new PhysicsUSDExporter();
  await expect(exporter.parseAsync(scene)).rejects.toThrow("untargeted drive");
  drive.setTarget({ position: 1, effort: 2 });
  await expect(exporter.parseAsync(scene)).rejects.toThrow("no effort term");
  joint.setDrive(
    new JointDrive({ stiffness: 10, maxForce: 5, maxVelocity: 2 }).setTarget({
      position: 1,
    }),
  );
  await expect(exporter.parseAsync(scene)).rejects.toThrow("no velocity limit");
  joint.setDrive(undefined);
  await expect(exporter.parseAsync(scene)).resolves.toBeInstanceOf(Uint8Array);
});

it("rejects drive data that has no matching joint-axis schema", () => {
  const text = `#usda 1.0
(
 metersPerUnit = 1
)
def PhysicsFixedJoint "Fixed" (
 prepend apiSchemas = ["PhysicsDriveAPI:angular"]
)
{
}`;
  expect(() => new PhysicsUSDLoader().parse(text)).toThrow(
    "Unsupported USD drive schema PhysicsDriveAPI:angular on prim /Fixed",
  );
});

it("reads an unbounded per-axis drive on a generic joint", () => {
  const { scene } = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Xform "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{
}
def PhysicsJoint "Joint" (
 prepend apiSchemas = ["PhysicsDriveAPI:rotX"]
)
{
 rel physics:body1 = </Body>
 float drive:rotX:physics:damping = 2
 float drive:rotX:physics:maxForce = inf
}`);
  const joint = scene.getObjectByName("Joint");
  if (!(joint instanceof GenericJoint)) throw new Error("Missing joint");
  expect(joint.getDrive("rotX")?.options.maxForce).toBeUndefined();
});
