import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  splitTransform,
  type PhysicsVelocity,
  type RigidBody,
  authoredVelocity,
  setWorldPose,
} from "@drawcall/physics";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import { refreshColliders } from "./colliders.js";

export interface BodyBinding {
  body: Rapier.RigidBody;
  /** The baseline `reset()` restores, captured at creation. */
  initialPose: Matrix4;
  initialVelocity: PhysicsVelocity;
  shapeKey: string;
  settingsVersion: number;
  /** The authored collider or mesh behind each Rapier collider handle. */
  sources: Map<number, Object3D>;
  scale: Vector3;
  colliderScales: Map<Object3D, Vector3>;
}

export function bodyPose(body: Rapier.RigidBody): Matrix4 {
  return new Matrix4().compose(
    new Vector3().copy(body.translation()),
    new Quaternion().copy(body.rotation()).normalize(),
    new Vector3(1, 1, 1),
  );
}

export function writePose(body: Rapier.RigidBody, pose: Matrix4): void {
  body.setTranslation(new Vector3().setFromMatrixPosition(pose), true);
  body.setRotation(new Quaternion().setFromRotationMatrix(pose), true);
}

export function synchronize(object: RigidBody, body: Rapier.RigidBody): void {
  setWorldPose(object, bodyPose(body));
}

export function createBody(
  api: typeof Rapier,
  backend: Rapier.World,
  object: RigidBody,
): BodyBinding {
  const options = object.options;
  const desc =
    object.bodyType === "static"
      ? api.RigidBodyDesc.fixed()
      : object.bodyType === "kinematic"
        ? api.RigidBodyDesc.kinematicPositionBased()
        : api.RigidBodyDesc.dynamic();
  const { pose, scale } = splitTransform(object.matrixWorld);
  const position = new Vector3().setFromMatrixPosition(pose);
  const velocity = authoredVelocity(object);
  desc
    .setTranslation(position.x, position.y, position.z)
    .setRotation(new Quaternion().setFromRotationMatrix(pose))
    .setCanSleep(options.canSleep)
    .setLinvel(velocity.linear.x, velocity.linear.y, velocity.linear.z)
    .setAngvel(velocity.angular);
  const body = backend.createRigidBody(desc);
  const binding: BodyBinding = {
    body,
    initialPose: pose,
    initialVelocity: velocity,
    scale,
    colliderScales: new Map(),
    shapeKey: "",
    settingsVersion: -1,
    sources: new Map(),
  };
  try {
    refreshBody(api, backend, object, binding);
    return binding;
  } catch (error) {
    backend.removeRigidBody(body);
    throw error;
  }
}

/** Follows authored collider and settings changes; scale is fixed at creation. */
export function refreshBody(
  api: typeof Rapier,
  backend: Rapier.World,
  object: RigidBody,
  binding: BodyBinding,
): void {
  if (splitTransform(object.matrixWorld).scale.distanceTo(binding.scale) > 1e-6)
    throw new Error(
      "Body scale cannot change after backend initialization; recreate the body",
    );
  refreshColliders(api, backend, object, binding);
  if (object.bodyType === "dynamic") assertDynamicMass(binding.body);
  if (object.settingsVersion === binding.settingsVersion) return;
  binding.body.setLinearDamping(object.linearDamping);
  binding.body.setAngularDamping(object.angularDamping);
  binding.body.setGravityScale(object.gravityScale, true);
  binding.settingsVersion = object.settingsVersion;
}

function assertDynamicMass(body: Rapier.RigidBody): void {
  const inertia = body.principalInertia();
  if (
    ![body.mass(), inertia.x, inertia.y, inertia.z].every(
      (value) => Number.isFinite(value) && value > 0,
    )
  )
    throw new Error("Dynamic body requires positive finite mass and inertia");
}
