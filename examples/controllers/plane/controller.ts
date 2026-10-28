import { Euler, MathUtils, Quaternion, Vector3 } from "three";
import type { PhysicsWorld } from "@drawcall/physics";
import { steer } from "../flight";
import { specification, type Plane } from "./model";

export class PlaneController {
  readonly input = { pitch: 0, roll: 0, throttle: 0.55, boost: false };
  private heading = 0;
  private readonly unsubscribe: () => void;

  constructor(
    private readonly world: PhysicsWorld,
    readonly plane: Plane,
  ) {
    this.reset();
    this.unsubscribe = world.onBeforeStep((delta) => this.update(delta));
  }

  get speed() {
    return this.plane.body.getVelocity().linear.length();
  }

  reset() {
    this.input.pitch = this.input.roll = 0;
    this.input.throttle = 0.55;
    this.input.boost = false;
    const rotation = this.plane.body.getWorldQuaternion(new Quaternion());
    this.heading = new Euler().setFromQuaternion(rotation, "YXZ").y;
  }

  dispose() {
    this.unsubscribe();
  }

  private update(delta: number) {
    const body = this.plane.body;
    const rotation = body.getWorldQuaternion(new Quaternion());
    const forward = new Vector3(0, 0, -1).applyQuaternion(rotation);
    const velocity = body.getVelocity().linear;
    const speed = Math.max(0, velocity.dot(forward));
    const throttle = MathUtils.clamp(this.input.throttle, 0, 1);
    const targetSpeed = 8 + throttle * 20 + (this.input.boost ? 12 : 0);
    const lift = MathUtils.smoothstep(speed, 4, 12);
    const acceleration = forward
      .clone()
      .multiplyScalar(MathUtils.clamp((targetSpeed - speed) * 2, -12, 10));
    // Arcade lift keeps cruise level; air grip turns velocity toward the nose.
    acceleration.addScaledVector(
      velocity.clone().addScaledVector(forward, -speed),
      -2,
    );
    acceleration.y -= this.world.gravity[1] * lift;
    body.applyForce(acceleration.multiplyScalar(specification.mass));

    this.heading -=
      MathUtils.clamp(this.input.roll, -1, 1) * 0.8 * lift * delta;
    const target = new Quaternion().setFromEuler(
      new Euler(
        MathUtils.clamp(this.input.pitch, -1, 1) * 0.55,
        this.heading,
        -MathUtils.clamp(this.input.roll, -1, 1) * 0.75,
        "YXZ",
      ),
    );
    steer(body, specification.inertia, target);
  }
}
