import { afterEach, expect, it } from "vitest";
import { Group } from "three";
import {
  registry,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

afterEach(() => registry.clear());

it("rejects untargeted and effort-driven export instead of changing actuation", async () => {
  const body = new RigidBody({ type: "static" });
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const drive = new JointDrive({ stiffness: 10 });
  joint.setDrive(drive);
  const scene = new Group().add(body, joint);
  const exporter = new PhysicsUSDExporter();
  await expect(exporter.parseAsync(scene)).rejects.toThrow("untargeted drive");
  drive.setTarget({ position: 1, effort: 2 });
  await expect(exporter.parseAsync(scene)).rejects.toThrow("no effort term");
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
