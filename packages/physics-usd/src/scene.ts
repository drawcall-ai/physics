import {
  Joint,
  RigidBody,
  constructLike,
  cleanup,
  rollback,
} from "@drawcall/physics";
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
      rollback(
        error,
        [() => target.dispose()],
        "Physics operation and cleanup failed",
      );
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
    const objects = [...this.owned];
    this.owned.clear();
    cleanup(
      [
        ...objects.map((object) => () => object.dispose()),
        () => this.removeFromParent(),
      ],
      "USD scene disposal failed",
    );
  }
}
