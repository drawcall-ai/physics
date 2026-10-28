import { Euler, MathUtils, Quaternion, Vector3 } from "three";
import type { PhysicsWorld } from "@drawcall/physics";
import { steer } from "../flight";
import { specification, type Quadcopter } from "./model";

export class QuadController {
  readonly input = { direction: new Vector3(), climb: 0, yaw: 0, boost: false };
  private heading = 0;
  private readonly unsubscribe: () => void;

  constructor(
    private readonly world: PhysicsWorld,
    readonly quad: Quadcopter,
  ) {
    this.reset();
    this.unsubscribe = world.onBeforeStep((delta) => this.update(delta));
  }

  get speed() {
    return this.quad.body.getVelocity().linear.length();
  }

  reset() {
    this.input.direction.set(0, 0, 0);
    this.input.climb = this.input.yaw = 0;
    this.input.boost = false;
    const rotation = this.quad.body.getWorldQuaternion(new Quaternion());
    this.heading = new Euler().setFromQuaternion(rotation, "YXZ").y;
  }

  dispose() {
    this.unsubscribe();
  }

  private update(delta: number) {
    const body = this.quad.body;
    const target = this.input.direction
      .clone()
      .setY(0)
      .clampLength(0, 1)
      .multiplyScalar(this.input.boost ? 12 : 7);
    target.y = MathUtils.clamp(this.input.climb, -1, 1) * 4;
    const acceleration = target
      .sub(body.getVelocity().linear)
      .multiplyScalar(4)
      .clampLength(0, 14);
    // The hover assist applies desired acceleration directly; tilt sells the maneuver.
    const force = acceleration.clone().sub(new Vector3(...this.world.gravity));
    body.applyForce(force.multiplyScalar(specification.mass));
    this.heading -= MathUtils.clamp(this.input.yaw, -1, 1) * 1.7 * delta;
    const up = new Vector3(0, 1, 0);
    const tilt = new Vector3(acceleration.x, 24, acceleration.z).normalize();
    const rotation = new Quaternion()
      .setFromUnitVectors(up, tilt)
      .multiply(new Quaternion().setFromAxisAngle(up, this.heading));
    steer(body, specification.inertia, rotation);
  }
}
