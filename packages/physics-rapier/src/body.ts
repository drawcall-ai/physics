import type { RigidBody as AuthoredBody, Collider } from "@drawcall/physics";
import type { RigidBody } from "@dimforge/rapier3d-compat";
import type * as Rapier from "@dimforge/rapier3d-compat";
import { collider } from "./shapes.js";
import { Matrix4, Quaternion, Vector3, type Object3D } from "three";
import {
  setWorldPose,
  initialVelocity,
  type PhysicsVelocity,
  splitTransform,
  resolveCollider,
} from "@drawcall/physics";

export function synchronize(object: AuthoredBody, body: RigidBody): void {
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
  shapes: string;
  settings: string;
  canSleep: boolean;
  scale: Vector3;
  colliderScales: Map<Object3D, Vector3>;
}

export function createBody(
  api: typeof Rapier,
  world: Rapier.World,
  object: AuthoredBody,
): BodyBinding {
  const options = object.options;
  const desc =
    options.type === "static"
      ? api.RigidBodyDesc.fixed()
      : options.type === "kinematic"
        ? api.RigidBodyDesc.kinematicPositionBased()
        : api.RigidBodyDesc.dynamic();
  const { pose, scale } = splitTransform(object.matrixWorld);
  const position = new Vector3().setFromMatrixPosition(pose);
  const velocity = initialVelocity(object);
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
    shapes: "",
    settings: "",
    canSleep: options.canSleep ?? true,
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
  object: AuthoredBody,
  binding: BodyBinding,
): void {
  const options = object.options;
  if (binding.canSleep !== (options.canSleep ?? true)) {
    throw new Error("canSleep cannot change after a body is created.");
  }
  if (splitTransform(object.matrixWorld).scale.distanceTo(binding.scale) > 1e-6)
    throw new Error(
      "Body scale cannot change after backend initialization; recreate the body",
    );
  const colliders = object.getColliders();
  const shapes = JSON.stringify([
    options.mass,
    colliders.map((shape) => shapeKey(shape, object)),
  ]);
  const body = binding.body;
  if (shapes !== binding.shapes) {
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
    const descriptors = resolved.map((shape) => collider(api, shape, object));
    const next: Rapier.Collider[] = [];
    try {
      for (const desc of descriptors)
        next.push(world.createCollider(desc, body));
      if (options.mass !== undefined) {
        const volume = next.reduce((sum, shape) => sum + shape.volume(), 0);
        if (volume <= 0)
          throw new Error(
            "Explicit mass requires colliders with positive volume.",
          );
        for (const shape of next)
          shape.setMass((options.mass * shape.volume()) / volume);
      }
    } catch (error) {
      for (const shape of next) world.removeCollider(shape, true);
      throw error;
    }
    const previous = body.numColliders() - next.length;
    for (let i = 0; i < previous; i++)
      world.removeCollider(body.collider(0), true);
    body.recomputeMassPropertiesFromColliders();
    body.wakeUp();
    binding.shapes = shapes;
    binding.colliderScales = new Map(
      resolved.map(({ collider, scale }) => [
        collider.source,
        binding.colliderScales.get(collider.source) ?? scale,
      ]),
    );
  }
  const settings = JSON.stringify([
    options.type,
    options.linearDamping,
    options.angularDamping,
    options.gravityScale,
    options.canSleep,
  ]);
  if (settings === binding.settings) return;
  const type =
    options.type === "static"
      ? api.RigidBodyType.Fixed
      : options.type === "kinematic"
        ? api.RigidBodyType.KinematicPositionBased
        : api.RigidBodyType.Dynamic;
  body.setBodyType(type, true);
  body.setLinearDamping(options.linearDamping ?? 0);
  body.setAngularDamping(options.angularDamping ?? 0);
  body.setGravityScale(options.gravityScale ?? 1, true);
  binding.settings = settings;
}

function shapeKey(source: Collider, body: AuthoredBody): unknown {
  const shape = source.shape();
  const { sensor, collisionGroups } = source;
  const material = body.getMaterial(source);
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
    .multiply(source.matrixWorld)
    .elements.map((value) => Math.round(value * 1e10) / 1e10);
  return [
    source.source.uuid,
    shapeData,
    transform,
    material,
    sensor,
    collisionGroups,
  ];
}
