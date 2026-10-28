import { Matrix4, Vector3, type Object3D } from "three";
import type { PhysicsWorld, RigidBody } from "@drawcall/physics";
import { CapsuleMotor } from "./motor";

interface Support {
  body: RigidBody;
  pose: Matrix4;
  normal: Vector3;
}

/** Shared locomotion policy. All collision and body access goes through @drawcall/physics. */
export class CharacterController {
  readonly input = {
    direction: new Vector3(),
    running: false,
    crouching: false,
  };
  readonly velocity = new Vector3();
  grounded = false;
  private readonly motor: CapsuleMotor;
  private readonly spawn: Vector3;
  private readonly standingHeight: number;
  private readonly crouchingHeight: number;
  private readonly unsubscribe: () => void;
  private support: Support | undefined;
  private readonly supportVelocity = new Vector3();
  private jumpBuffer = 0;
  private coyoteTime = 0;

  constructor(
    private readonly world: PhysicsWorld,
    object: Object3D,
    options: { radius?: number; height?: number } = {},
  ) {
    const radius = options.radius ?? 0.35;
    this.standingHeight = options.height ?? 1.8;
    if (
      !Number.isFinite(radius) ||
      !Number.isFinite(this.standingHeight) ||
      radius <= 0 ||
      this.standingHeight <= radius * 2
    )
      throw new Error("Character height must exceed twice its positive radius");
    this.crouchingHeight = Math.min(
      this.standingHeight,
      Math.max(radius * 2 + 0.1, this.standingHeight * 0.62),
    );
    if (
      world.gravity[0] !== 0 ||
      world.gravity[2] !== 0 ||
      world.gravity[1] >= 0
    )
      throw new Error("This example requires downward Y gravity");
    this.motor = new CapsuleMotor(world, object, radius, this.standingHeight);
    this.spawn = this.motor.position.clone();
    // Query completed body poses so translating and rotating supports work in either engine.
    this.unsubscribe = world.onAfterStep((delta) => this.update(delta));
  }

  get height(): number {
    return this.motor.height;
  }
  get crouched(): boolean {
    return this.height < this.standingHeight;
  }
  get speed(): number {
    return Math.hypot(this.velocity.x, this.velocity.z);
  }
  get groundName(): string {
    return this.support?.body.name || (this.grounded ? "Ground" : "Air");
  }

  jump(): void {
    this.jumpBuffer = 0.12;
  }

  reset(position: Vector3 = this.spawn): void {
    this.motor.position.copy(position);
    this.motor.height = this.standingHeight;
    this.velocity.set(0, 0, 0);
    this.input.direction.set(0, 0, 0);
    this.input.running = false;
    this.input.crouching = false;
    this.support = undefined;
    this.supportVelocity.set(0, 0, 0);
    this.grounded = false;
    this.jumpBuffer = 0;
    this.coyoteTime = 0;
    this.motor.commit();
  }

  dispose(): void {
    this.unsubscribe();
  }

  private update(delta: number): void {
    this.transport(delta);
    this.motor.recover();
    this.motor.resize(
      this.input.crouching ? this.crouchingHeight : this.standingHeight,
    );
    this.jumpBuffer = Math.max(0, this.jumpBuffer - delta);
    this.coyoteTime = this.grounded
      ? 0.12
      : Math.max(0, this.coyoteTime - delta);

    const target = this.input.direction.clone().setY(0).clampLength(0, 1);
    target.multiplyScalar(this.crouched ? 2.2 : this.input.running ? 6 : 3.8);
    const horizontal = this.velocity.clone().setY(0);
    const change = target
      .sub(horizontal)
      .clampLength(0, (this.grounded ? 35 : 9) * delta);
    this.velocity.add(change);
    const jumping = this.jumpBuffer > 0 && this.coyoteTime > 0;
    if (jumping) {
      this.velocity.y = 7.5;
      this.jumpBuffer = 0;
      this.coyoteTime = 0;
      this.grounded = false;
      this.support = undefined;
    }
    this.velocity.y += this.world.gravity[1] * delta;
    const wasGrounded = this.grounded;
    const rising = this.velocity.y > 0.1 && !wasGrounded;
    const motion = this.velocity.clone();
    if (wasGrounded && this.support)
      motion.setY(0).projectOnPlane(this.support.normal).setLength(this.speed);
    const normals = this.motor.move(
      motion.multiplyScalar(delta),
      wasGrounded && !jumping,
    );
    for (const normal of normals) {
      if (normal.y >= Math.SQRT1_2) {
        this.velocity.y = Math.max(0, this.velocity.y);
        continue;
      }
      const inward = this.velocity.dot(normal);
      if (inward < 0) this.velocity.addScaledVector(normal, -inward);
      if (wasGrounded && normal.y > 0)
        this.velocity.y = Math.min(0, this.velocity.y);
    }

    const ground =
      !jumping && !rising ? this.motor.ground(wasGrounded ? 0.3 : 0.025) : null;
    this.grounded = ground !== null;
    this.support = ground
      ? {
          body: ground.body,
          pose: ground.body.matrixWorld.clone(),
          normal: ground.normal,
        }
      : undefined;
    if (ground) this.velocity.y = 0;
    // transport() already applied this step's platform motion. Inherit it for subsequent
    // airborne steps, including walking off an edge, without moving twice on takeoff.
    if (jumping || (wasGrounded && !ground))
      this.velocity.add(this.supportVelocity);
    this.motor.commit();
  }

  private transport(delta: number): void {
    this.supportVelocity.set(0, 0, 0);
    const support = this.support;
    if (!support || support.body.world !== this.world) return;
    support.body.updateWorldMatrix(true, false);
    const carried = this.motor.position
      .clone()
      .applyMatrix4(support.pose.clone().invert())
      .applyMatrix4(support.body.matrixWorld);
    const displacement = carried.sub(this.motor.position);
    this.supportVelocity.copy(displacement).divideScalar(delta);
    this.motor.carry(displacement, support.body);
  }
}
