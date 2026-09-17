import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  ancestorBody,
  resolveCollider,
  resolveCollisionGroups,
  splitTransform,
  type Collider,
  type RigidBody,
  type Trigger,
} from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import type { BodyBinding } from "./body.js";
import { shapeFingerprint } from "./colliders.js";
import { descriptor } from "./shapes.js";

export interface TriggerBinding {
  key: string;
  colliders: Rapier.Collider[];
  scales: Map<Collider, Vector3>;
  standalone?: Rapier.RigidBody;
}

export function refreshTrigger(
  api: typeof Rapier,
  backend: Rapier.World,
  object: Trigger,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
  previous?: TriggerBinding,
): TriggerBinding {
  const shapes = object.getColliders();
  const parent = ancestorBody(object);
  const frame = parent ?? object;
  const needsCarrier = !parent || parent.bodyType === "static";
  const key = JSON.stringify([
    object.collisionGroups,
    parent?.uuid,
    needsCarrier ? object.matrixWorld.elements : undefined,
    shapes.map((shape) => shapeFingerprint(shape, frame)),
  ]);
  if (previous?.key === key) return previous;
  const body = parent ? bodies.get(parent)?.body : undefined;
  if (parent && !body) throw new Error("Missing trigger's prepared body");
  const scales = new Map<Collider, Vector3>();
  const descriptors = shapes.map((collider) => {
    const part = resolveCollider(frame, collider);
    const captured = previous?.scales.get(collider);
    if (captured && captured.distanceTo(part.scale) > 1e-6)
      throw new Error(
        "Trigger collider scale cannot change after initialization",
      );
    scales.set(collider, part.scale);
    if (needsCarrier)
      part.matrix.premultiply(splitTransform(frame.matrixWorld).pose);
    const position = new Vector3().setFromMatrixPosition(part.matrix);
    const rotation = new Quaternion().setFromRotationMatrix(part.matrix);
    const { membership, filter } = resolveCollisionGroups(collider, object);
    return descriptor(api, part.shape)
      .setTranslation(position.x, position.y, position.z)
      .setRotation(rotation)
      .setDensity(0)
      .setSensor(true)
      .setCollisionGroups(((membership << 16) | filter) >>> 0)
      .setActiveCollisionTypes(api.ActiveCollisionTypes.ALL);
  });
  const colliders: Rapier.Collider[] = [];
  // Rapier omits fixed/fixed pairs, even sensors. A private kinematic carrier
  // keeps standalone regions observable against every target body type.
  const standalone = !needsCarrier
    ? undefined
    : (previous?.standalone ??
      backend.createRigidBody(api.RigidBodyDesc.kinematicPositionBased()));
  try {
    for (const desc of descriptors)
      colliders.push(backend.createCollider(desc, standalone ?? body));
  } catch (error) {
    for (const collider of colliders) backend.removeCollider(collider, true);
    if (standalone && standalone !== previous?.standalone)
      backend.removeRigidBody(standalone);
    throw error;
  }
  if (previous) {
    for (const collider of previous.colliders)
      backend.removeCollider(collider, true);
    if (previous.standalone && previous.standalone !== standalone)
      backend.removeRigidBody(previous.standalone);
  }
  return { key, colliders, scales, standalone };
}

export function removeTrigger(
  backend: Rapier.World,
  binding: TriggerBinding,
): void {
  for (const collider of binding.colliders)
    if (collider.isValid()) backend.removeCollider(collider, true);
  if (binding.standalone) backend.removeRigidBody(binding.standalone);
}
