import * as THREE from "three";
import {
  BoxCollider,
  CylinderCollider,
  RigidBody,
  PrismaticJoint,
  RevoluteJoint,
  FixedJoint,
  type PhysicsWorld,
  type Vec3,
} from "@drawcall/physics";

export const simulationOptions = {
  fixedDelta: 1 / 120,
  maxSubsteps: 12,
  solverIterations: 32,
};

// A stiff, torque-limited steering servo must overcome stationary tire scrub.
export const steeringMotor = {
  stiffness: 300000,
  damping: 1500,
  maxForce: 4000,
};

export const specification = {
  mass: 950,
  wheelRadius: 0.36,
  wheelbase: 2.7,
  track: 1.8,
  stiffness: 35000,
  damping: 3200,
  torque: 480,
  droop: 0.16,
  bump: 0.22,
};
const paint = new THREE.MeshStandardMaterial({
  color: "#eaa65a",
  metalness: 0.35,
  roughness: 0.35,
});
const metal = new THREE.MeshStandardMaterial({
  color: "#aab7c4",
  metalness: 0.75,
  roughness: 0.3,
});
const rubber = new THREE.MeshStandardMaterial({
  color: "#20252b",
  roughness: 0.95,
});
const glass = new THREE.MeshStandardMaterial({
  color: "#233f51",
  metalness: 0.45,
  roughness: 0.2,
});

export function box(size: Vec3, material: THREE.Material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
  return mesh;
}

export function createCar(world: PhysicsWorld) {
  const root = new THREE.Group();
  root.name = "SuspensionCar";
  function body(
    name: string,
    mass: number,
    position: Vec3,
    size: Vec3,
    collides = false,
  ) {
    const result = new RigidBody({ world, mass, canSleep: false });
    result.name = name;
    result.position.set(...position);
    result.add(
      new BoxCollider({
        size,
        collisionGroups: { membership: 2, filter: collides ? 1 : 0 },
      }),
    );
    root.add(result);
    return result;
  }
  const chassis = body(
    "Chassis",
    specification.mass,
    [0, 0.94, 0],
    [1.5, 0.32, 3.9],
    true,
  );
  chassis.add(box([1.5, 0.32, 3.9], paint));
  const cabin = box([1.3, 0.54, 1.65], glass);
  cabin.position.set(0, 0.43, -0.15);
  chassis.add(cabin);
  const wheels = [];
  for (const front of [true, false])
    for (const side of [-1, 1]) {
      const x = (side * specification.track) / 2;
      const z = ((front ? 1 : -1) * specification.wheelbase) / 2;
      const name = `${front ? "Front" : "Rear"}${side < 0 ? "Left" : "Right"}`;
      const carrier = body(
        `${name}Carrier`,
        18,
        [x, 0.42, z],
        [0.12, 0.16, 0.12],
      );
      const knuckle = body(
        `${name}Knuckle`,
        8,
        [x, 0.42, z],
        [0.12, 0.12, 0.12],
      );
      const spring = new PrismaticJoint({
        body0: chassis,
        body1: carrier,
        axis: "Y",
        frame0: new THREE.Matrix4().makeTranslation(x, -0.52, z),
        frame1: new THREE.Matrix4(),
        limits: [-specification.droop, specification.bump],
        drive: {
          stiffness: specification.stiffness,
          damping: specification.damping,
          targetPosition: 0,
        },
      });
      spring.name = `${name}Suspension`;
      const steeringFrames = {
        body0: carrier,
        body1: knuckle,
        frame0: new THREE.Matrix4(),
        frame1: new THREE.Matrix4(),
      };
      const steering = front
        ? new RevoluteJoint({
            ...steeringFrames,
            axis: "Y",
            limits: [-0.6, 0.6],
            drive: { ...steeringMotor },
          })
        : new FixedJoint(steeringFrames);
      steering.name = `${name}${front ? "Steering" : "KnuckleMount"}`;
      const tire = new RigidBody({
        world,
        mass: 22,
        canSleep: false,
        material: { dynamicFriction: 1.2, staticFriction: 1.2, restitution: 0 },
      });
      tire.name = `${name}Wheel`;
      tire.position.copy(carrier.position);
      const collider = new CylinderCollider({
        radius: specification.wheelRadius,
        height: 0.26,
        collisionGroups: { membership: 2, filter: 1 },
      });
      collider.rotation.z = Math.PI / 2;
      tire.add(collider);
      const wheel = new THREE.Mesh(
        new THREE.CylinderGeometry(
          specification.wheelRadius,
          specification.wheelRadius,
          0.26,
          32,
        ),
        rubber,
      );
      wheel.rotation.z = Math.PI / 2;
      tire.add(wheel);
      const spoke = box([0.29, 0.045, 0.43], metal);
      tire.add(spoke);
      const axle = new RevoluteJoint({
        body0: knuckle,
        body1: tire,
        axis: "X",
        frame0: new THREE.Matrix4(),
        frame1: new THREE.Matrix4(),
        drive: { targetVelocity: 0, damping: 80, maxForce: 0 },
      });
      axle.name = `${name}Motor`;
      // Spring geometry is visual only; force comes from the prismatic joint.
      const points = Array.from(
        { length: 97 },
        (_, i) =>
          new THREE.Vector3(
            0.065 * Math.cos((i / 96) * Math.PI * 14),
            i / 96,
            0.065 * Math.sin((i / 96) * Math.PI * 14),
          ),
      );
      const coil = new THREE.Mesh(
        new THREE.TubeGeometry(
          new THREE.CatmullRomCurve3(points),
          96,
          0.012,
          5,
          false,
        ),
        paint,
      );
      const damper = new THREE.Mesh(
        new THREE.CylinderGeometry(0.025, 0.025, 1, 8),
        metal,
      );
      root.add(tire, spring, steering, axle, coil, damper);
      wheels.push({
        name,
        front,
        side,
        x,
        z,
        carrier,
        tire,
        spring,
        steering,
        axle,
        coil,
        damper,
      });
    }
  return { root, chassis, wheels };
}

export type Car = ReturnType<typeof createCar>;
