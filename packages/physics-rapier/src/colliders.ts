import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  resolveCollider,
  geometryVersion,
  type Collider,
  type RigidBody,
} from "@drawcall/physics";
import { Quaternion, Vector3, type Object3D } from "three";
import type { BodyBinding } from "./body.js";
import { colliderDescs } from "./shapes.js";

/** Rebuilds the body's colliders when their authored shapes, placement, materials, or settings changed. */
export function refreshColliders(
  api: typeof Rapier,
  backend: Rapier.World,
  object: RigidBody,
  binding: BodyBinding,
): void {
  const colliders = object.getColliders();
  const shapeKey = JSON.stringify([
    object.materialVersion,
    object.collisionGroups,
    colliders.map((collider) => shapeFingerprint(collider, object)),
  ]);
  if (shapeKey === binding.shapeKey) return;
  const { options } = object;
  const { body } = binding;
  const resolved = colliders.map((collider) =>
    resolveCollider(
      object,
      collider,
      binding.colliderScales.get(collider.source),
    ),
  );
  assertCapturedScales(object, binding, resolved);
  const completeMass = options.centerOfMass !== undefined;
  // Each authored collider becomes one or more Rapier colliders.
  const parts = resolved.flatMap((part) =>
    colliderDescs(api, part, object).map(
      (desc) =>
        [
          completeMass ? desc.setDensity(0) : desc,
          part.collider.source,
        ] as const,
    ),
  );
  const created = createColliders(backend, object, body, parts, completeMass);
  for (const handle of binding.sources.keys()) {
    const collider = backend.getCollider(handle);
    if (collider) backend.removeCollider(collider, true);
  }
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
    created.map(([collider, source]) => [collider.handle, source]),
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

function assertCapturedScales(
  object: RigidBody,
  binding: BodyBinding,
  resolved: ReturnType<typeof resolveCollider>[],
): void {
  for (const { collider, scale } of resolved) {
    const captured = binding.colliderScales.get(collider.source);
    if (captured && captured.distanceTo(scale) > 1e-6)
      throw new Error(
        `Collider scale cannot change after backend initialization: ${object.name}/${collider.name || collider.type} (${captured.toArray()} → ${scale.toArray()}); recreate the body`,
      );
  }
}

/** Creates the colliders, each with its authored source, and their mass settled, or none. */
function createColliders(
  backend: Rapier.World,
  object: RigidBody,
  body: Rapier.RigidBody,
  parts: readonly (readonly [Rapier.ColliderDesc, Object3D])[],
  completeMass: boolean,
): [Rapier.Collider, Object3D][] {
  const { options } = object;
  const pairs: [Rapier.Collider, Object3D][] = [];
  const created: Rapier.Collider[] = [];
  try {
    for (const [desc, source] of parts) {
      const collider = backend.createCollider(desc, body);
      created.push(collider);
      pairs.push([collider, source]);
    }
    if (options.mass !== undefined && created.length && !completeMass)
      distributeMass(created, options.mass);
    if (
      object.bodyType === "dynamic" &&
      options.mass === undefined &&
      totalMass(created) <= 0
    )
      throw new Error("Dynamic body requires positive mass and inertia");
    return pairs;
  } catch (error) {
    for (const collider of created) backend.removeCollider(collider, true);
    throw error;
  }
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

export function shapeFingerprint(collider: Collider, owner: Object3D): unknown {
  const shape = collider.shape();

  const shapeData =
    shape.kind === "mesh"
      ? {
          kind: shape.kind,
          approximation: shape.approximation,
          version: geometryVersion(shape.geometry),
        }
      : shape;
  // Relative transforms accumulate tiny roundoff as bodies move under parents.
  const transform = owner.matrixWorld
    .clone()
    .invert()
    .multiply(collider.matrixWorld)
    .elements.map((value) => Math.round(value * 1e10) / 1e10);
  return [collider.source.uuid, shapeData, transform, collider.settingsVersion];
}
