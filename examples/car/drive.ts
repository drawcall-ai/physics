import * as THREE from "three";
import { RevoluteJoint, type PhysicsWorld } from "@drawcall/physics";
import { specification, steeringControl, type Car } from "./model";

export function driveCar(world: PhysicsWorld, car: Car) {
  const input = {
    throttle: 0,
    steer: 0,
    brake: 0,
    automatic: true,
  };
  const body = car.chassis;
  const wheels = car.wheels;
  let steering = 0;
  let completed = false;
  const steeringIntegral = new Map<RevoluteJoint, number>();
  const telemetry = {
    speed: 0,
    completed: false,
  };
  const unsubscribe = world.onBeforeStep((dt) => {
    const elapsed = world.time + dt;
    const forward = new THREE.Vector3(0, 0, 1).applyQuaternion(
      car.chassis.quaternion,
    );
    const speed = body.getVelocity().linear.dot(forward);
    const z = car.chassis.position.z;
    if (z >= 59) completed = true;
    const automaticBrake = elapsed < 1 || completed;
    const throttle = input.automatic
      ? automaticBrake
        ? 0
        : THREE.MathUtils.clamp((6 - speed) * 0.6, 0, 1)
      : input.throttle;
    const brake = input.automatic ? (automaticBrake ? 1 : 0) : input.brake;
    // Heading and lane feedback keep the unattended run on the straight course.
    const steer = input.automatic
      ? THREE.MathUtils.clamp(
          -car.chassis.position.x * 0.15 -
            Math.atan2(forward.x, forward.z) * 1.5,
          -0.3,
          0.3,
        )
      : (input.steer * 0.45) / (1 + Math.abs(speed) * 0.06);
    steering = THREE.MathUtils.damp(steering, steer, 7, dt);
    for (const wheel of wheels) {
      const angle =
        Math.abs(steering) < 0.001
          ? 0
          : Math.atan(
              specification.wheelbase /
                (specification.wheelbase / Math.tan(steering) - wheel.x),
            );
      const suspension = wheel.spring.getState();
      wheel.spring.setEffort(
        -specification.stiffness * suspension.position -
          specification.damping * suspension.velocity,
      );
      if (wheel.steering instanceof RevoluteJoint) {
        const state = wheel.steering.getState();
        const integral = THREE.MathUtils.clamp(
          (steeringIntegral.get(wheel.steering) ?? 0) +
            (angle - state.position) * dt,
          -steeringControl.integralLimit,
          steeringControl.integralLimit,
        );
        steeringIntegral.set(wheel.steering, integral);
        wheel.steering.setEffort(
          THREE.MathUtils.clamp(
            steeringControl.stiffness * (angle - state.position) +
              steeringControl.integral * integral -
              steeringControl.damping * state.velocity,
            -steeringControl.maxTorque,
            steeringControl.maxTorque,
          ),
        );
      }
      const targetVelocity = brake ? 0 : Math.sign(throttle) * 55;
      const torque = brake
        ? brake * 1100
        : Math.abs(throttle) * specification.torque;
      wheel.axle.setEffort(
        THREE.MathUtils.clamp(
          (targetVelocity - wheel.axle.getState().velocity) *
            (brake ? 450 : 80),
          -torque,
          torque,
        ),
      );
    }
    const velocity = body.getVelocity().linear;
    body.applyForce(
      velocity.clone().multiplyScalar(-0.45 * velocity.length() - 10),
    );
  });
  const after = world.onAfterStep(() => {
    telemetry.speed = body
      .getVelocity()
      .linear.dot(
        new THREE.Vector3(0, 0, 1).applyQuaternion(car.chassis.quaternion),
      );
    telemetry.completed = completed && Math.abs(telemetry.speed) < 0.2;
  });
  return {
    input,
    telemetry,
    reset() {
      completed = false;
      steering = 0;
      steeringIntegral.clear();
      input.throttle = input.steer = input.brake = 0;
      world.reset();
    },
    dispose() {
      unsubscribe();
      after();
    },
    updateVisuals() {
      for (const wheel of wheels) {
        const bottom = wheel.carrier.position;
        const top = car.chassis.localToWorld(
          new THREE.Vector3(wheel.x, 0.08, wheel.z),
        );
        const direction = top.sub(bottom);
        const length = direction.length();
        wheel.coil.position.copy(bottom);
        wheel.coil.quaternion.setFromUnitVectors(
          new THREE.Vector3(0, 1, 0),
          direction.normalize(),
        );
        wheel.coil.scale.y = length;
        wheel.damper.position
          .copy(bottom)
          .addScaledVector(direction, length / 2);
        wheel.damper.quaternion.copy(wheel.coil.quaternion);
        wheel.damper.scale.y = length;
      }
    },
  };
}
