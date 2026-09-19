import { Joint, RigidBody, constructLike } from "@drawcall/physics";
import { Group } from "three";
import type { Vec3 } from "@drawcall/physics";

export class PhysicsUSDScene extends Group {
  gravity: Vec3 = [0, -9.81, 0];
  private readonly owned = new Set<RigidBody | Joint>();

  override clone(recursive = true): this {
    const target = constructLike(this, []);
    try {
      return target.copy(this, recursive);
    } catch (error) {
      target.dispose();
      throw error;
    }
  }

  override copy(source: this, recursive = true): this {
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
    this.removeFromParent();
  }
}
