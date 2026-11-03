import { Quaternion, Vector3 } from "three";
import type { RigidBody, Vec3 } from "@drawcall/physics";

/** A damped attitude assist: collisions still act on the body, and correction takes time. */
export function steer(body: RigidBody, inertia: Vec3, target: Quaternion) {
  const rotation = body.getWorldQuaternion(new Quaternion());
  const inverse = rotation.clone().invert();
  const error = target.clone().multiply(inverse);
  const sign = error.w < 0 ? -1 : 1;
  const acceleration = new Vector3(error.x, error.y, error.z)
    .multiplyScalar(48 * sign)
    .addScaledVector(body.getVelocity().angular, -8)
    .clampLength(0, 24);
  const torque = acceleration
    .applyQuaternion(inverse)
    .multiply(new Vector3(...inertia))
    .applyQuaternion(rotation);
  body.applyTorque(torque);
}
