import type { RigidBody, Collider } from "@drawcall/physics";
import type * as Rapier from "@dimforge/rapier3d-compat";
import { colliderDesc } from "./shapes.js";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import {
  setWorldPose,
  authoredVelocity,
  type PhysicsVelocity,
  splitTransform,
  resolveCollider,
} from "@drawcall/physics";

export function synchronize(object: RigidBody, body: Rapier.RigidBody): void {
  setWorldPose(
    object,
    new Matrix4().compose(
      new Vector3().copy(body.translation()),
      new Quaternion().copy(body.rotation()).normalize(),
      new Vector3(1, 1, 1),
    ),
  );
}

export interface BodyBinding {
  body: Rapier.RigidBody;
  initial: Matrix4;
  velocity: PhysicsVelocity;
  shapeKey: string;
  settingsVersion: number;
  sources: Map<number, Object3D>;
  scale: Vector3;
  colliderScales: Map<Object3D, Vector3>;
}

export function createBody(
  api: typeof Rapier,
  world: Rapier.World,
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
    .setCanSleep(options.canSleep ?? true)
    .setLinvel(velocity.linear.x, velocity.linear.y, velocity.linear.z)
    .setAngvel(velocity.angular);
  const body = world.createRigidBody(desc);
  const binding: BodyBinding = {
    body,
    initial: pose,
    velocity,
    scale,
    colliderScales: new Map(),
    shapeKey: "",
    settingsVersion: -1,
    sources: new Map(),
  };
  try {
    refreshBody(api, world, object, binding);
    return binding;
  } catch (error) {
    world.removeRigidBody(body);
    throw error;
  }
}

export function refreshBody(
  api: typeof Rapier,
  world: Rapier.World,
  object: RigidBody,
  binding: BodyBinding,
): void {
  const options = object.options;
  if (splitTransform(object.matrixWorld).scale.distanceTo(binding.scale) > 1e-6)
    throw new Error(
      "Body scale cannot change after backend initialization; recreate the body",
    );
  const colliders = object.getColliders();
  const shapeKey = JSON.stringify([
    object.materialVersion,
    colliders.map((collider) => shapeFingerprint(collider, object)),
  ]);
  const body = binding.body;
  if (shapeKey !== binding.shapeKey) {
    const resolved = colliders.map((collider) =>
      resolveCollider(
        object,
        collider,
        binding.colliderScales.get(collider.source),
      ),
    );
    for (const { collider, scale } of resolved) {
      const captured = binding.colliderScales.get(collider.source);
      if (captured && captured.distanceTo(scale) > 1e-6)
        throw new Error(
          `Collider scale cannot change after backend initialization: ${object.name}/${collider.name || collider.type} (${captured.toArray()} → ${scale.toArray()}); recreate the body`,
        );
    }
    const completeMass = options.centerOfMass !== undefined;
    const descriptors = resolved.map((shape) => {
      const desc = colliderDesc(api, shape, object);
      return completeMass ? desc.setDensity(0) : desc;
    });
    const next: Rapier.Collider[] = [];
    try {
      for (const desc of descriptors)
        next.push(world.createCollider(desc, body));
      if (options.mass !== undefined && next.length && !completeMass) {
        let inferredMass = next.reduce((sum, shape) => sum + shape.mass(), 0);
        // Zero-density shapes still have volume; unit density recovers the ratios that split the explicit mass.
        if (inferredMass === 0) {
          for (const shape of next) shape.setDensity(1);
          inferredMass = next.reduce((sum, shape) => sum + shape.mass(), 0);
        }
        if (inferredMass <= 0)
          throw new Error(
            "Inferring mass properties requires colliders with positive volume",
          );
        for (const shape of next)
          shape.setMass((options.mass * shape.mass()) / inferredMass);
      }
      if (
        object.bodyType === "dynamic" &&
        options.mass === undefined &&
        next.reduce((sum, shape) => sum + shape.mass(), 0) <= 0
      )
        throw new Error("Dynamic body requires positive mass and inertia");
    } catch (error) {
      for (const shape of next) world.removeCollider(shape, true);
      throw error;
    }
    const previous = body.numColliders() - next.length;
    for (let i = 0; i < previous; i++)
      world.removeCollider(body.collider(0), true);
    if (completeMass) {
      body.setAdditionalMassProperties(
        options.mass,
        new Vector3(...options.centerOfMass),
        new Vector3(...options.diagonalInertia),
        new Quaternion(...(options.principalAxes ?? [0, 0, 0, 1])),
        true,
      );
    }
    body.recomputeMassPropertiesFromColliders();
    binding.sources = new Map(
      next.map((shape, index) => {
        const source = colliders[index];
        if (!source) throw new Error("Missing authored collider");
        return [shape.handle, source.source];
      }),
    );
    body.wakeUp();
    binding.shapeKey = shapeKey;
    binding.colliderScales = new Map(
      resolved.map(({ collider, scale }) => [
        collider.source,
        binding.colliderScales.get(collider.source) ?? scale,
      ]),
    );
  }
  if (object.bodyType === "dynamic") {
    const inertia = body.principalInertia();
    if (
      ![body.mass(), inertia.x, inertia.y, inertia.z].every(
        (value) => Number.isFinite(value) && value > 0,
      )
    )
      throw new Error("Dynamic body requires positive finite mass and inertia");
  }
  if (object.settingsVersion === binding.settingsVersion) return;
  body.setLinearDamping(object.linearDamping);
  body.setAngularDamping(object.angularDamping);
  body.setGravityScale(object.gravityScale, true);
  binding.settingsVersion = object.settingsVersion;
}

function shapeFingerprint(collider: Collider, body: RigidBody): unknown {
  const shape = collider.shape();

  const shapeData =
    shape.kind === "mesh"
      ? {
          kind: shape.kind,
          approximation: shape.approximation,
          positions: Array.from(shape.geometry.getAttribute("position").array),
          indices: shape.geometry.index
            ? Array.from(shape.geometry.index.array)
            : null,
        }
      : shape;
  // Relative transforms accumulate tiny roundoff as bodies move under parents.
  const transform = body.matrixWorld
    .clone()
    .invert()
    .multiply(collider.matrixWorld)
    .elements.map((value) => Math.round(value * 1e10) / 1e10);
  return [collider.source.uuid, shapeData, transform, collider.settingsVersion];
}
