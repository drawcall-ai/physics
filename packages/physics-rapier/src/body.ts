import type * as Rapier from "@dimforge/rapier3d-compat";
import { splitTransform, type RigidBody } from "@drawcall/physics";
import {
  authoredVelocity,
  lockScale,
  rollback,
  setWorldPose,
  type Initial,
} from "@drawcall/physics/backend";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import {
  colliderDescs,
  fingerprint,
  removeColliders,
  resolve,
} from "./shapes.js";

export class BodyBinding {
  readonly native: Rapier.RigidBody;
  readonly initial: Initial;
  readonly scale: Vector3;
  /** The authored collider or mesh behind each Rapier collider. */
  sources = new Map<Rapier.Collider, Object3D>();
  scales = new Map<Object3D, Vector3>();
  fingerprint = "";
  version = -1;

  constructor(api: typeof Rapier, simulation: Rapier.World, object: RigidBody) {
    object.validate();
    const { pose, scale } = splitTransform(object.matrixWorld);
    this.initial = { pose, velocity: authoredVelocity(object) };
    const { linear, angular } = this.initial.velocity;
    this.scale = scale;
    const desc =
      object.bodyType === "static"
        ? api.RigidBodyDesc.fixed()
        : object.bodyType === "kinematic"
          ? api.RigidBodyDesc.kinematicPositionBased()
          : api.RigidBodyDesc.dynamic();
    const position = new Vector3().setFromMatrixPosition(pose);
    desc
      .setTranslation(position.x, position.y, position.z)
      .setRotation(new Quaternion().setFromRotationMatrix(pose))
      .setCanSleep(object.options.canSleep)
      .setLinvel(linear.x, linear.y, linear.z)
      .setAngvel(angular);
    this.native = simulation.createRigidBody(desc);
  }
}

/** Follows authored collider and settings changes; scale is fixed as the body joins. */
export function prepareBody(
  api: typeof Rapier,
  simulation: Rapier.World,
  object: RigidBody,
  binding: BodyBinding,
): void {
  object.validate();
  const { scale } = splitTransform(object.matrixWorld);
  lockScale(object.name || object.type, binding.scale, scale);
  prepareColliders(api, simulation, object, binding);
  if (object.bodyType === "dynamic") assertDynamicMass(binding.native);
  if (object.version === binding.version) return;
  binding.native.setLinearDamping(object.linearDamping);
  binding.native.setAngularDamping(object.angularDamping);
  binding.native.setGravityScale(object.gravityScale, true);
  binding.version = object.version;
}

/** Copies the Rapier pose to the scene. */
export function writeBack(object: RigidBody, body: Rapier.RigidBody): void {
  setWorldPose(object, bodyPose(body));
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

/**
 * Rebuilds the colliders when their shapes, placement, materials or settings changed. A rebuild
 * that fails keeps the previous colliders.
 */
function prepareColliders(
  api: typeof Rapier,
  simulation: Rapier.World,
  object: RigidBody,
  binding: BodyBinding,
): void {
  const colliders = object.getColliders();
  const current = JSON.stringify([
    object.material,
    object.collisionGroups,
    colliders.map((collider) => fingerprint(collider, object)),
  ]);
  if (current === binding.fingerprint) return;
  const { options } = object;
  const body = binding.native;
  const resolved = colliders.map((collider) =>
    resolve(object, collider, binding.scales),
  );
  // Explicit mass properties replace what the colliders would contribute.
  const explicit = options.centerOfMass !== undefined;
  const sources = new Map<Rapier.Collider, Object3D>();
  try {
    for (const part of resolved)
      for (const desc of colliderDescs(api, part, object)) {
        const collider = simulation.createCollider(
          explicit ? desc.setDensity(0) : desc,
          body,
        );
        sources.set(collider, part.collider.source);
      }
    const created = [...sources.keys()];
    if (options.mass !== undefined && created.length && !explicit)
      distributeMass(created, options.mass);
    if (
      object.bodyType === "dynamic" &&
      options.mass === undefined &&
      totalMass(created) <= 0
    )
      throw new Error("Dynamic body requires positive mass and inertia");
  } catch (error) {
    rollback(
      error,
      [() => removeColliders(simulation, sources)],
      "Collider rebuild rollback failed",
    );
  }
  removeColliders(simulation, binding.sources);
  if (explicit)
    body.setAdditionalMassProperties(
      options.mass,
      new Vector3(...options.centerOfMass),
      new Vector3(...options.diagonalInertia),
      new Quaternion(...(options.principalAxes ?? [0, 0, 0, 1])),
      true,
    );
  body.recomputeMassPropertiesFromColliders();
  body.wakeUp();
  binding.sources = sources;
  binding.scales = new Map(
    resolved.map(({ collider, scale }) => [collider.source, scale]),
  );
  binding.fingerprint = current;
}

/** Splits an explicit total mass across the colliders in proportion to their volume. */
function distributeMass(colliders: Rapier.Collider[], mass: number): void {
  let inferred = totalMass(colliders);
  // Zero-density shapes still have volume; unit density recovers the ratios that split the explicit mass.
  if (inferred === 0) {
    for (const collider of colliders) collider.setDensity(1);
    inferred = totalMass(colliders);
  }
  if (inferred <= 0)
    throw new Error(
      "Inferring mass properties requires colliders with positive volume",
    );
  for (const collider of colliders)
    collider.setMass((mass * collider.mass()) / inferred);
}

function totalMass(colliders: Rapier.Collider[]): number {
  return colliders.reduce((sum, collider) => sum + collider.mass(), 0);
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
