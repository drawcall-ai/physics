import type { Object3D } from "three";
import { SoftBody } from "@drawcall/physics";
import { input } from "./input.js";
import { parse } from "./model.js";
import { source, type Source } from "./source.js";
import type { FEMInput, FEMOptions } from "./types.js";

/**
 * Prepares a static, closed rest mesh as a reduced SoftBody that displays a clone of it; the object stays caller-owned.
 * Scale and local mesh transforms are baked. Every call prepares anew; CachedFEMLoader caches components.
 */
export async function prepareSoftBody(
  object: Object3D,
  options: FEMOptions,
): Promise<SoftBody> {
  const snapshot = source(object);
  const recipe = input(snapshot, options);
  return build(snapshot, recipe, await reduce(recipe));
}

export async function reduce(recipe: FEMInput): Promise<Uint8Array> {
  const { prepare } = await import("./prepare/index.js");
  return prepare(recipe);
}

/** Builds the body from a reduction of `recipe`, rejecting one that does not match it. */
export function build(
  snapshot: Source,
  recipe: FEMInput,
  bytes: Uint8Array,
): SoftBody {
  const model = parse(JSON.parse(new TextDecoder().decode(bytes)));
  if (
    !equal(model.renderPositions, recipe.positions) ||
    !equal(model.renderTriangles, recipe.triangles) ||
    !equal(model.mountRegion.min, recipe.mount.min) ||
    !equal(model.mountRegion.max, recipe.mount.max) ||
    !equal(model.contactRegion.min, recipe.contact.min) ||
    !equal(model.contactRegion.max, recipe.contact.max)
  )
    throw new Error(
      "FEM reduction does not match the source geometry or physical recipe",
    );
  const body = new SoftBody({ model });
  body.setMaterial(recipe.material);
  body.add(snapshot.visual);
  for (const skin of snapshot.skins)
    body.bindGeometry(skin.mesh, skin.vertices);
  body.position.setFromMatrixPosition(snapshot.pose);
  body.quaternion.setFromRotationMatrix(snapshot.pose);
  body.name = snapshot.visual.name;
  return body;
}

function equal(left: readonly number[], right: readonly number[]): boolean {
  return (
    left.length === right.length &&
    left.every((value, index) => value === right[index])
  );
}
