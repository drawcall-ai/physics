import { Group } from "three";
import type { Vec3 } from "@drawcall/physics";

export class PhysicsUSDScene extends Group {
  gravity: Vec3 = [0, -9.81, 0];

  override copy(source: this, recursive = true): this {
    super.copy(source, recursive);
    this.gravity = [...source.gravity];
    return this;
  }
}
