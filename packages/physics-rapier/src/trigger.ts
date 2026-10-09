import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  ancestorBody,
  splitTransform,
  type RigidBody,
  type Trigger,
} from "@drawcall/physics";
import type { Object3D, Vector3 } from "three";
import type { BodyBinding } from "./body.js";
import {
  descriptor,
  fingerprint,
  place,
  removeColliders,
  resolve,
} from "./shapes.js";

export class TriggerBinding {
  key = "";
  /** The authored collider behind each Rapier sensor handle. */
  sources = new Map<number, Object3D>();
  scales = new Map<Object3D, Vector3>();
  /**
   * Rapier omits fixed/fixed pairs, even sensors. A private kinematic carrier keeps a trigger
   * outside any moving body observable against every target body type.
   */
  carrier: Rapier.RigidBody | undefined = undefined;
}

/** Rebuilds the sensors when their shapes, placement, groups or carrying body changed. */
export function prepareTrigger(
  api: typeof Rapier,
  native: Rapier.World,
  object: Trigger,
  binding: TriggerBinding,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
): void {
  const colliders = object.getColliders();
  const parent = ancestorBody(object);
  const frame = parent ?? object;
  const moving = parent?.bodyType === "static" ? undefined : parent;
  const key = JSON.stringify([
    object.collisionGroups,
    parent?.uuid,
    moving ? undefined : object.matrixWorld.elements,
    colliders.map((collider) => fingerprint(collider, frame)),
  ]);
  if (key === binding.key) return;
  const resolved = colliders.map((collider) =>
    resolve(frame, collider, binding.scales),
  );
  removeColliders(native, binding.sources);
  if (moving && binding.carrier) {
    native.removeRigidBody(binding.carrier);
    binding.carrier = undefined;
  }
  const body = moving
    ? bodies.get(moving)?.native
    : (binding.carrier ??= native.createRigidBody(
        api.RigidBodyDesc.kinematicPositionBased(),
      ));
  if (!body) throw new Error("Trigger's body is outside the world");
  for (const { collider, shape, matrix } of resolved) {
    if (!moving) matrix.premultiply(splitTransform(frame.matrixWorld).pose);
    const desc = place(descriptor(api, shape), matrix, collider, object)
      .setDensity(0)
      .setSensor(true)
      .setActiveCollisionTypes(api.ActiveCollisionTypes.ALL);
    binding.sources.set(
      native.createCollider(desc, body).handle,
      collider.source,
    );
  }
  binding.scales = new Map(
    resolved.map(({ collider, scale }) => [collider.source, scale]),
  );
  binding.key = key;
}

export function releaseTrigger(
  native: Rapier.World,
  binding: TriggerBinding,
): void {
  removeColliders(native, binding.sources);
  if (binding.carrier) native.removeRigidBody(binding.carrier);
}
