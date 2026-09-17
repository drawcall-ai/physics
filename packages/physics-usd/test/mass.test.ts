import { expect, it } from "vitest";
import { PrismaticJoint, RigidBody } from "@drawcall/physics";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";

it("roundtrips independent USD mass properties, velocities and a linear braking drive", async () => {
  const imported = new PhysicsUSDLoader().parse(`#usda 1.0
(
 metersPerUnit = 1
)
def Xform "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{
 float physics:mass = 2
 point3f physics:centerOfMass = (1, 2, 3)
 float3 physics:diagonalInertia = (3, 4, 5)
 quatf physics:principalAxes = (0.8, 0, 0.6, 0)
 vector3f physics:velocity = (1, 2, 3)
 vector3f physics:angularVelocity = (0, 180, 0)
 double3 xformOp:scale = (2, 3, 4)
 uniform token[] xformOpOrder = ["xformOp:scale"]
}
def PhysicsPrismaticJoint "Slider" (
 prepend apiSchemas = ["PhysicsDriveAPI:linear"]
)
{
 rel physics:body1 = </Body>
 float physics:lowerLimit = -2
 float physics:upperLimit = 3
 bool physics:jointEnabled = false
 bool physics:collisionEnabled = true
 float drive:linear:physics:damping = 3
 float drive:linear:physics:maxForce = inf
}`);
  const scenes = [imported];
  try {
    scenes.push(
      new PhysicsUSDLoader().parse(
        await new PhysicsUSDExporter().parseAsync(imported),
      ),
    );
    for (const scene of scenes) {
      const body = scene.getObjectByName("Body");
      const joint = scene
        .getObjectsByProperty("isObject3D", true)
        .find((object) => object instanceof PrismaticJoint);
      if (!(body instanceof RigidBody) || !(joint instanceof PrismaticJoint))
        throw new Error("Missing body or joint");
      expect(body.getColliders()).toEqual([]);
      expect(body.options.mass).toBe(2);
      expect(body.options.centerOfMass).toEqual([2, 6, 12]);
      expect(body.options.diagonalInertia).toEqual([3, 4, 5]);
      expect(body.options.principalAxes).toEqual([0, 0.6, 0, 0.8]);
      expect(body.getVelocity().linear.toArray()).toEqual([1, 2, 3]);
      expect(body.getVelocity().angular.y).toBeCloseTo(Math.PI);
      expect(joint.limits).toEqual([-2, 3]);
      expect(joint.enabled).toBe(false);
      expect(joint.collideConnected).toBe(true);
      expect(joint.drive?.options.model).toBe("force");
      expect(joint.drive?.options.stiffness).toBe(0);
      expect(joint.drive?.options.damping).toBe(3);
      expect(joint.drive?.options.maxForce).toBeUndefined();
      expect(joint.drive?.target).toEqual({
        position: 0,
        velocity: 0,
        effort: 0,
      });
    }
  } finally {
    for (const scene of scenes) scene.dispose();
  }
});

it.each(["mass", "centerOfMass", "diagonalInertia"])(
  "rejects external explicit mass properties missing %s",
  (missing) => {
    const properties = {
      mass: "float physics:mass = 2",
      centerOfMass: "point3f physics:centerOfMass = (0, 0, 0)",
      diagonalInertia: "float3 physics:diagonalInertia = (1, 1, 1)",
    };
    const text = `#usda 1.0
(
 metersPerUnit = 1
)
def Xform "Body" (
 prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{
${Object.entries(properties)
  .filter(([name]) => name !== missing)
  .map(([, value]) => value)
  .join("\n")}
}`;
    expect(() => new PhysicsUSDLoader().parse(text)).toThrow(
      "Explicit mass properties require mass, centerOfMass and diagonalInertia",
    );
  },
);
