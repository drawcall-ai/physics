import { AuthoringWorld, Joint, RigidBody } from "@drawcall/physics";
import { Group } from "three";
import type { PhysicsWorld, Vec3 } from "@drawcall/physics";

export class PhysicsUSDScene extends Group {
  readonly world: PhysicsWorld;
  gravity: Vec3 = [0, -9.81, 0];
  private readonly owned = new Set<RigidBody | Joint>();
  private readonly ownsWorld: boolean;

  constructor(world?: PhysicsWorld) {
    super();
    this.world = world ?? new AuthoringWorld();
    this.ownsWorld = world === undefined;
  }

  override clone(recursive = true): this {
    const target: unknown = Reflect.construct(this.constructor, [this.world]);
    if (!this.isClone(target))
      throw new Error(
        "USD scene clone constructor returned an incompatible object",
      );
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
  }

  private isClone(value: unknown): value is this {
    return (
      value instanceof PhysicsUSDScene &&
      Object.getPrototypeOf(value) === Object.getPrototypeOf(this)
    );
  }

  override copy(source: this, recursive = true): this {
    if (this.world !== source.world)
      throw new Error("USD scene copies must share their physics world");
    super.copy(source, recursive);
    this.gravity = [...source.gravity];
    this.traverse((object) => {
      if (object instanceof RigidBody || object instanceof Joint)
        this.own(object);
    });
    return this;
  }

  own(object: RigidBody | Joint): void {
    this.owned.add(object);
  }

  dispose(): void {
    this.traverse((object) => {
      if (object instanceof RigidBody || object instanceof Joint)
        this.own(object);
    });
    for (const object of this.owned) object.dispose();
    this.owned.clear();
    if (this.ownsWorld) this.world.dispose();
    this.removeFromParent();
  }
}
