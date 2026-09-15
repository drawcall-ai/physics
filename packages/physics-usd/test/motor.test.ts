import { afterEach, beforeEach, expect, it } from "vitest";
import { Group } from "three";
import {
  AuthoringWorld,
  JointMotor,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
});
afterEach(() => world.dispose());

it("rejects disabled and untargeted motor export instead of changing actuation", async () => {
  const body = new RigidBody({ world, type: "static" });
  const joint = new RevoluteJoint({ body0: null, body1: body });
  const motor = new JointMotor({ joint, stiffness: 10 });
  const scene = new Group().add(body, joint);
  const exporter = new PhysicsUSDExporter();
  await expect(exporter.parseAsync(scene)).rejects.toThrow(
    "disabled or untargeted motor",
  );
  motor.setTarget({ position: 1 }).setEnabled(false);
  await expect(exporter.parseAsync(scene)).rejects.toThrow(
    "disabled or untargeted motor",
  );
  motor.dispose();
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
