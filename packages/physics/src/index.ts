export {
  type RigidBodyEventMap,
  type RigidBodyType,
  type MassProperties,
  type RigidBodyOptions,
  RigidBody,
  ancestorBody,
} from "./body.js";
export {
  type Vec3,
  type AutoColliders,
  type PhysicsMaterial,
  type CollisionGroups,
  Collider,
  type Shape,
  BoxCollider,
  SphereCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
} from "./colliders.js";
export { Joint, type JointOptions } from "./joint.js";
export {
  FixedJoint,
  ScalarJoint,
  type AxisJointOptions,
  AxisJoint,
  RevoluteJoint,
  PrismaticJoint,
  SphericalJoint,
  type DistanceJointOptions,
  DistanceJoint,
} from "./joints.js";
export {
  type JointDof,
  jointDofs,
  type DofMotion,
  type GenericJointOptions,
  GenericJoint,
} from "./generic.js";
export {
  type PhysicsOptions,
  type PhysicsVelocity,
  type AxisJointState,
  type SphericalJointState,
  type DistanceJointState,
  type JointReading,
  type RaycastOptions,
  type RaycastHit,
  type PhysicsWorld,
} from "./world.js";
export { clone } from "./clone.js";
export { splitTransform } from "./transforms.js";
export { resolveCollider } from "./shapes.js";
export {
  JointDrive,
  type JointDriveOptions,
  type JointDriveTarget,
} from "./drive.js";

// Adapter integration; scene code needs none of these.
export * from "./backend.js";

export { Trigger, type TriggerEventMap } from "./trigger.js";

export { registry } from "./registry.js";
