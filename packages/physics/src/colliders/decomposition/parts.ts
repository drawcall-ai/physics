/**
 * Convex parts of closed triangle meshes, for backends that cannot collide a concave mesh:
 * every triangle mesh in MuJoCo, and those on moving bodies in Rapier. Each geometry version is
 * decomposed once, in a module worker in browser bundles. Until its parts arrive a mesh collides as
 * its convex hull, and `partsVersion` changes when they arrive, so backends rebuild it.
 */
import type { BufferGeometry, Object3D, Vector3 } from "three";
import { RigidBody } from "../../rigid/body.js";
import type { Collider } from "../collider.js";
import { geometryVersion, vertices } from "../geometry.js";
import { closedMesh } from "./mesh.js";
import { run } from "./run.js";

/** Selects the triangle meshes a backend collides as convex parts rather than as triangles. */
export type Decomposes = (body: RigidBody, geometry: BufferGeometry) => boolean;

/** A geometry version's decomposition. */
interface Job {
  version: string;
  parts?: number[][];
  error?: Error;
  /**
   * Resolves once `parts` or `error` is set, or once a newer job replaced this one before its
   * turn; never rejects.
   */
  done: Promise<void>;
}
/** Each geometry's latest decomposition. */
const jobs = new WeakMap<BufferGeometry, Job>();
/** Decompositions take turns, in order. */
let queue: Promise<unknown> = Promise.resolve();

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
      if (!settled(job)) waits.add(job.done);
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
  owner: RigidBody,
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
 * The geometry version, which also changes once its convex parts arrive or fail. Backends
 * fingerprint colliders made of convex parts by it, so those are rebuilt from the hull; other
 * mesh colliders use `geometryVersion`.
 */
export function partsVersion(geometry: BufferGeometry): string {
  const version = geometryVersion(geometry);
  const job = jobs.get(geometry);
  return job?.version === version && settled(job)
    ? `${version}/settled`
    : version;
}

/**
 * The job for the geometry's current version, started if there is none. A mesh that is not
 * closed fails at once; a job replaced before its turn is skipped.
 */
function jobOf(geometry: BufferGeometry): Job {
  const version = geometryVersion(geometry);
  const known = jobs.get(geometry);
  if (known?.version === version) return known;
  const job: Job = { version, done: Promise.resolve() };
  jobs.set(geometry, job);
  try {
    const { positions, indices } = closedMesh(geometry);
    const turn = queue.then(() =>
      jobs.get(geometry) === job ? run(positions, indices) : undefined,
    );
    // The job records its own failure; the next one takes its turn regardless.
    queue = turn.catch(() => {});
    job.done = turn.then(
      (parts) => {
        job.parts = parts;
      },
      (error: unknown) => {
        job.error = asError(error);
      },
    );
  } catch (error) {
    job.error = asError(error);
  }
  return job;
}

function settled(job: Job): boolean {
  return job.parts !== undefined || job.error !== undefined;
}

function asError(error: unknown): Error {
  return error instanceof Error ? error : new Error(String(error));
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

function failure(owner: RigidBody, collider: Collider, error: Error): Error {
  const name = `${owner.name || owner.type}/${collider.source.name || collider.source.type}`;
  const message = `Convex decomposition failed for ${name}: ${error.message}`;
  return new Error(message, { cause: error });
}
