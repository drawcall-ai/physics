import type { Object3D, Vector3 } from "three";
import type { PhysicsWorld, PhysicsWorldOptions } from "@drawcall/physics";

/** The page lifecycle shared by the four independent controller examples. */
export interface Demo {
  root: Object3D;
  target: Object3D;
  description: string;
  keys: readonly { code: string; label: string }[];
  camera: Vector3;
  options: Omit<PhysicsWorldOptions, "scene">;
  start(world: PhysicsWorld): {
    readInput(keys: ReadonlySet<string>, forward: Vector3): void;
    press(code: string): void;
    update(): string;
    reset(): void;
    dispose(): void;
  };
}

export function held(keys: ReadonlySet<string>, ...codes: string[]): boolean {
  return codes.some((code) => keys.has(code));
}

export function direction(keys: ReadonlySet<string>) {
  return {
    x:
      Number(held(keys, "KeyD", "ArrowRight")) -
      Number(held(keys, "KeyA", "ArrowLeft")),
    z:
      Number(held(keys, "KeyW", "ArrowUp")) -
      Number(held(keys, "KeyS", "ArrowDown")),
  };
}
