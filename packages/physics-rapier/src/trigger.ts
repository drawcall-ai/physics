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
  fingerprint = "";
  /** The authored collider behind each Rapier sensor. */
  sources = new Map<Rapier.Collider, Object3D>();
  scales = new Map<Object3D, Vector3>();
  /**
   * Rapier omits fixed/fixed pairs, even sensors. A private kinematic carrier keeps a trigger
   * outside any moving body observable against every target body type.
   */
  carrier: Rapier.RigidBody | undefined = undefined;
}

/**
 * Rebuilds the sensors when their shapes, placement, groups or carrying body changed, or when a
 * carrying body that left took them along.
 */
export function prepareTrigger(
  api: typeof Rapier,
  simulation: Rapier.World,
  object: Trigger,
  binding: TriggerBinding,
  bodies: ReadonlyMap<RigidBody, BodyBinding>,
): void {
  const colliders = object.getColliders();
  const parent = ancestorBody(object);
  const frame = parent ?? object;
  const moving = parent?.bodyType === "static" ? undefined : parent;
  const current = JSON.stringify([
    object.collisionGroups,
    parent?.uuid,
    moving ? undefined : object.matrixWorld.elements,
    colliders.map((collider) => fingerprint(collider, frame)),
  ]);
  const intact = [...binding.sources.keys()].every((sensor) =>
    sensor.isValid(),
  );
  if (current === binding.fingerprint && intact) return;
  const resolved = colliders.map((collider) =>
    resolve(frame, collider, binding.scales),
  );
  removeColliders(simulation, binding.sources);
  if (moving && binding.carrier) {
    simulation.removeRigidBody(binding.carrier);
    binding.carrier = undefined;
  }
  const body = moving
    ? bodies.get(moving)?.native
    : (binding.carrier ??= simulation.createRigidBody(
        api.RigidBodyDesc.kinematicPositionBased(),
      ));
  if (!body) throw new Error("Trigger's body is outside the world");
  for (const { collider, shape, matrix } of resolved) {
    if (!moving) matrix.premultiply(splitTransform(frame.matrixWorld).pose);
    const desc = place(descriptor(api, shape), matrix, collider, object)
      .setDensity(0)
      .setSensor(true)
      .setActiveCollisionTypes(api.ActiveCollisionTypes.ALL);
    binding.sources.set(simulation.createCollider(desc, body), collider.source);
  }
  binding.scales = new Map(
    resolved.map(({ collider, scale }) => [collider.source, scale]),
  );
  binding.fingerprint = current;
}

export function releaseTrigger(
  simulation: Rapier.World,
  binding: TriggerBinding,
): void {
  removeColliders(simulation, binding.sources);
  if (binding.carrier) simulation.removeRigidBody(binding.carrier);
}
