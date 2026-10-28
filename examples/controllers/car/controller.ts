import { MathUtils, Quaternion, Vector3 } from "three";
import { RigidBody, type PhysicsWorld } from "@drawcall/physics";
import { tuning, type Car } from "./model";

/** Four suspension rays and bounded tire forces: a small, tunable arcade car. */
export class CarController {
  readonly input = { throttle: 0, steer: 0, brake: 0, handbrake: false };
  speed = 0;
  contacts = 0;
  private steering = 0;
  private readonly unsubscribe: () => void;

  constructor(
    private readonly world: PhysicsWorld,
    private readonly car: Car,
  ) {
    this.unsubscribe = world.onBeforeStep((delta) => this.update(delta));
  }

  get grounded(): boolean {
    return this.contacts > 0;
  }

  reset(): void {
    this.input.throttle = this.input.steer = this.input.brake = 0;
    this.input.handbrake = false;
    this.speed = this.steering = this.contacts = 0;
    for (const wheel of this.car.wheels) {
      wheel.pivot.position.copy(wheel.mount).y -= tuning.suspension;
      wheel.pivot.rotation.y = 0;
      wheel.spin.rotation.x = 0;
    }
  }

  dispose(): void {
    this.unsubscribe();
  }

  private update(delta: number): void {
    const { body, wheels } = this.car;
    body.updateWorldMatrix(true, false);
    const rotation = body.getWorldQuaternion(new Quaternion());
    const forward = new Vector3(0, 0, -1).applyQuaternion(rotation);
    const down = new Vector3(0, -1, 0).applyQuaternion(rotation);
    const velocity = body.getVelocity().linear;
    this.speed = velocity.dot(forward);
    const steer = MathUtils.clamp(this.input.steer, -1, 1);
    this.steering = MathUtils.damp(
      this.steering,
      (-steer * tuning.steerAngle) / (1 + Math.abs(this.speed) * 0.045),
      9,
      delta,
    );
    this.contacts = 0;
    for (const wheel of wheels) {
      const origin = body.localToWorld(wheel.mount.clone());
      const hit = this.world.raycast(
        origin,
        down,
        tuning.suspension + tuning.radius,
        {
          excludeBodies: [body],
        },
      );
      wheel.pivot.rotation.y = wheel.front ? this.steering : 0;
      wheel.pivot.position.y = wheel.mount.y - tuning.suspension;
      if (!hit || hit.kind !== "body" || !(hit.body instanceof RigidBody))
        continue;
      if (hit.normal.dot(down) > -0.25) continue;
      this.contacts++;
      const length = Math.max(0, hit.distance - tuning.radius);
      wheel.pivot.position.y = wheel.mount.y - length;
      const relative = body
        .getVelocityAtPoint(hit.point)
        .sub(hit.body.getVelocityAtPoint(hit.point));
      const load = MathUtils.clamp(
        (tuning.suspension - length) * tuning.spring -
          relative.dot(hit.normal) * tuning.damping,
        0,
        tuning.mass * Math.abs(this.world.gravity[1]),
      );
      const suspension = hit.normal.clone().multiplyScalar(load);
      body.applyForce(suspension, origin);
      const heading = new Vector3(0, 0, -1)
        .applyAxisAngle(new Vector3(0, 1, 0), wheel.pivot.rotation.y)
        .applyQuaternion(rotation)
        .projectOnPlane(hit.normal)
        .normalize();
      const side = new Vector3().crossVectors(heading, hit.normal);
      const longitudinal = relative.dot(heading);
      const lateral = relative.dot(side);
      const throttle = MathUtils.clamp(this.input.throttle, -1, 1);
      const topSpeed = throttle < 0 ? tuning.reverseSpeed : tuning.forwardSpeed;
      const drive =
        throttle *
        tuning.driveForce *
        Math.max(
          0,
          1 - Math.max(0, this.speed * Math.sign(throttle)) / topSpeed,
        );
      const brake = Math.max(
        MathUtils.clamp(this.input.brake, 0, 1),
        !wheel.front && this.input.handbrake ? 1 : 0,
      );
      const stopping = MathUtils.clamp(
        (-longitudinal * tuning.mass) / (4 * delta),
        -brake * tuning.brakeForce,
        brake * tuning.brakeForce,
      );
      const grip =
        !wheel.front && this.input.handbrake ? tuning.driftGrip : tuning.grip;
      const tire = heading
        .multiplyScalar(drive + stopping - longitudinal * 25)
        .addScaledVector(side, -lateral * tuning.cornering)
        .clampLength(0, load * grip);
      // Apply grip near the COM to keep arcade cornering stable without an upright constraint.
      const tirePoint = origin.clone().addScaledVector(down, 0.08);
      body.applyForce(tire, tirePoint);
      if (hit.body.bodyType === "dynamic")
        hit.body.applyForce(suspension.add(tire).negate(), hit.point);
      wheel.spin.rotation.x -= (longitudinal * delta) / tuning.radius;
    }
    body.applyForce(velocity.multiplyScalar(-velocity.length() * 0.5));
  }
}
