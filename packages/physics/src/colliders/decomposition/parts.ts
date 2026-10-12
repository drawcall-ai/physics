/**
 * Convex parts of closed triangle meshes, for backends that cannot collide a concave mesh:
 * every triangle mesh in MuJoCo, and those on moving bodies in Rapier. Each geometry version is
 * decomposed once, in a module worker in browser bundles. Until its parts arrive a mesh collides as
 * its convex hull, and `partsVersion` changes when they arrive, so backends rebuild it.
 */
import type { BufferGeometry, Object3D, Vector3 } from "three";
import { RigidBody } from "../../body.js";
import type { Trigger } from "../../trigger.js";
import type { Collider } from "../collider.js";
import { geometryVersion, vertices } from "../geometry.js";
import { closedMesh } from "./mesh.js";
import { schedule } from "./thread.js";

/** Selects the triangle meshes a backend collides as convex parts rather than as triangles. */
export type Decomposes = (body: RigidBody, geometry: BufferGeometry) => boolean;

/** A geometry version's decomposition, which becomes the geometry's job as it starts. */
class Job {
  parts?: number[][];
  error?: Error;
  /**
   * Resolves once `parts` or `error` is set, or once a newer job replaced this one before it
   * ran; never rejects.
   */
  readonly done: Promise<void>;
  constructor(
    readonly version: string,
    geometry: BufferGeometry,
  ) {
    // Current before it is scheduled, since the queue skips jobs that are not.
    jobs.set(geometry, this);
    this.done = this.run(geometry);
  }
  get settled(): boolean {
    return this.parts !== undefined || this.error !== undefined;
  }
  /** A mesh that is not closed fails before this returns. */
  private async run(geometry: BufferGeometry): Promise<void> {
    try {
      const { positions, indices } = closedMesh(geometry);
      this.parts = await schedule(
        positions,
        indices,
        () => jobs.get(geometry) === this,
      );
    } catch (error) {
      this.error = error instanceof Error ? error : new Error(String(error));
    }
  }
}
/** Each geometry's latest decomposition. */
const jobs = new WeakMap<BufferGeometry, Job>();

/**
 * Decomposes the triangle mesh colliders of the bodies under `root` that `needs` selects, and
 * resolves once all of them have parts for their current geometry, including meshes that join
 * or change meanwhile. Rejects if decomposing one failed.
 */
export async function settle(root: Object3D, needs: Decomposes): Promise<void> {
  for (;;) {
    const waits = new Set<Promise<void>>();
    for (const { body, collider, geometry } of trimeshes(root, needs)) {
      const job = jobOf(geometry);
      if (job.error) throw failure(body, collider, job.error);
      if (!job.settled) waits.add(job.done);
    }
    if (!waits.size) return;
    await Promise.all(waits);
  }
}

/**
 * A triangle mesh collider's convex parts at `scale`, as flat vertex lists. Until the parts of
 * its geometry's current version arrive, this starts decomposing it and returns every vertex as
 * one part, which backends collide as the convex hull. Throws if decomposing it failed.
 */
export function convexParts(
  owner: RigidBody | Trigger,
  collider: Collider,
  scale: Vector3,
): number[][] {
  const geometry = trimesh(collider);
  if (!geometry)
    throw new Error("Only triangle mesh colliders have convex parts");
  const job = jobOf(geometry);
  if (job.error) throw failure(owner, collider, job.error);
  return (job.parts ?? [vertices(geometry)]).map((part) =>
    part.map((value, i) => value * scale.getComponent(i % 3)),
  );
}

/**
 * The geometry version, which also changes once its convex parts arrive or fail; fingerprints
 * of colliders made from convex parts hold it, so they are rebuilt from the hull.
 */
export function partsVersion(geometry: BufferGeometry): string {
  const version = geometryVersion(geometry);
  const job = jobs.get(geometry);
  return job?.version === version && job.settled
    ? `${version}/settled`
    : version;
}

/** The job for the geometry's current version, started if there is none. */
function jobOf(geometry: BufferGeometry): Job {
  const version = geometryVersion(geometry);
  const known = jobs.get(geometry);
  return known?.version === version ? known : new Job(version, geometry);
}

function trimesh(collider: Collider): BufferGeometry | undefined {
  const shape = collider.shape();
  return shape.kind === "mesh" && shape.approximation === "trimesh"
    ? shape.geometry
    : undefined;
}

/** The triangle mesh colliders of the bodies under `root` that `needs` selects. */
function trimeshes(root: Object3D, needs: Decomposes) {
  const found: {
    body: RigidBody;
    collider: Collider;
    geometry: BufferGeometry;
  }[] = [];
  root.traverse((body) => {
    if (!(body instanceof RigidBody)) return;
    for (const collider of body.getColliders()) {
      const geometry = trimesh(collider);
      if (geometry && needs(body, geometry))
        found.push({ body, collider, geometry });
    }
  });
  return found;
}

function failure(
  owner: RigidBody | Trigger,
  collider: Collider,
  error: Error,
): Error {
  const name = `${owner.name || owner.type}/${collider.source.name || collider.source.type}`;
  const message = `Convex decomposition failed for ${name}: ${error.message}`;
  return new Error(message, { cause: error });
}
