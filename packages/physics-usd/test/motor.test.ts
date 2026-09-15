import { afterEach, beforeEach, expect, it } from "vitest";
import { Group } from "three";
import {
  AuthoringWorld,
  JointMotor,
  PrismaticJoint,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { strFromU8, unzipSync } from "fflate";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

let world: AuthoringWorld;
beforeEach(() => {
  world = new AuthoringWorld();
});
afterEach(() => world.dispose());

it.each(["force", "acceleration"] as const)(
  "roundtrips %s motors with angular units and persistent targets",
  async (model) => {
    const body = new RigidBody({
      world,
      mass: 2,
      centerOfMass: [0, 0, 0],
      diagonalInertia: [1, 1, 1],
      colliders: false,
    });
    const hinge = new RevoluteJoint({
      body0: null,
      body1: body,
      limits: [-Math.PI / 2, Math.PI / 2],
    });
    const motor = new JointMotor({
      joint: hinge,
      model,
      stiffness: 100,
      damping: 10,
      maxForce: 20,
    });
    motor.setTarget({ position: Math.PI / 4, velocity: Math.PI / 2 });
    const scene = new Group().add(body, hinge);
    const bytes = await new PhysicsUSDExporter().parseAsync(scene);
    const layer = unzipSync(bytes)["model.usda"];
    if (!layer) throw new Error("Missing USD layer");
    const text = strFromU8(layer);
    expect(text).toContain("PhysicsDriveAPI:angular");
    expect(text).toContain("drive:angular:physics:targetPosition = 45");
    expect(text).toContain("drive:angular:physics:targetVelocity = 90");
    const imported = new PhysicsUSDLoader().parse(bytes);
    try {
      const joint = imported
        .getObjectsByProperty("isObject3D", true)
        .find((object) => object instanceof RevoluteJoint);
      if (!(joint instanceof RevoluteJoint) || !joint.motor)
        throw new Error("Missing imported motor");
      expect(joint.motor.options.joint).toBe(joint);
      expect(joint.motor.options.model).toBe(model);
      expect(joint.motor.options.stiffness).toBeCloseTo(100);
      expect(joint.motor.options.damping).toBeCloseTo(10);
      expect(joint.motor.options.maxForce).toBe(20);
      expect(joint.motor.target?.position).toBeCloseTo(Math.PI / 4);
      expect(joint.motor.target?.velocity).toBeCloseTo(Math.PI / 2);
      expect(joint.limits).toEqual(hinge.limits);
      expect(joint.motor.enabled).toBe(true);
      const ownedMotor = joint.motor;
      imported.dispose();
      expect(ownedMotor.disposed).toBe(true);
      expect(motor.disposed).toBe(false);
    } finally {
      imported.dispose();
    }
  },
);

it.each(["7", "inf"])(
  "imports an independent linear USD drive with maxForce %s and default targets",
  (maxForce) => {
    const scene = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Cube "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
)
{
}
def PhysicsPrismaticJoint "Slider" (
 prepend apiSchemas = ["PhysicsDriveAPI:linear"]
)
{
 rel physics:body1 = </Body>
 float drive:linear:physics:stiffness = 12
 float drive:linear:physics:damping = 3
 float drive:linear:physics:maxForce = ${maxForce}
}`);
    try {
      const joint = scene.getObjectByName("Slider");
      if (!(joint instanceof PrismaticJoint) || !joint.motor)
        throw new Error("Missing motor");
      expect(joint.motor.options.model).toBe("force");
      expect(joint.motor.options.stiffness).toBe(12);
      expect(joint.motor.options.damping).toBe(3);
      expect(joint.motor.options.maxForce).toBe(
        maxForce === "inf" ? undefined : 7,
      );
      expect(joint.motor.target).toEqual({ position: 0, velocity: 0 });
    } finally {
      scene.dispose();
    }
  },
);

it("roundtrips velocity braking and unbounded force", async () => {
  const body = new RigidBody({ world, type: "static" });
  const slider = new PrismaticJoint({ body0: null, body1: body });
  new JointMotor({ joint: slider, damping: 5 }).setTarget({ velocity: 0 });
  const imported = new PhysicsUSDLoader().parse(
    await new PhysicsUSDExporter().parseAsync(new Group().add(body, slider)),
  );
  try {
    const joint = imported
      .getObjectsByProperty("isObject3D", true)
      .find((object) => object instanceof PrismaticJoint);
    if (!(joint instanceof PrismaticJoint) || !joint.motor)
      throw new Error("Missing motor");
    expect(joint.motor.options.maxForce).toBeUndefined();
    expect(joint.motor.options.damping).toBe(5);
    expect(joint.motor.options.stiffness ?? 0).toBe(0);
    expect(joint.motor.target?.velocity).toBe(0);
  } finally {
    imported.dispose();
  }
});

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
