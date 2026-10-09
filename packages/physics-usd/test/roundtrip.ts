import type { Object3D } from "three";
import { PhysicsUSDExporter, PhysicsUSDLoader } from "../src/index.js";
import type { PhysicsUSDExportOptions } from "../src/index.js";

export async function roundtrip(
  scene: Object3D,
  options?: PhysicsUSDExportOptions,
) {
  return new PhysicsUSDLoader().parseAsync(
    await new PhysicsUSDExporter().parseAsync(scene, options),
  );
}
