import type * as Rapier from "@dimforge/rapier3d-compat";
import {
  type PhysicsVelocity,
  type RigidBody,
  authoredVelocity,
} from "@drawcall/physics";
import { Vector3 } from "three";
import { createBody, type BodyBinding } from "./body.js";

export type Command = (body: Rapier.RigidBody) => void;

interface Queue {
  /** The authored velocity when queuing began; commands replay on top of it in call order. */
  velocity: PhysicsVelocity;
  changesVelocity: boolean;
  commands: Command[];
}

/**
 * Commands for bodies that have no backend body yet. They replay once the body is
 * prepared; until then, reads that depend on them run in a throwaway world.
 */
export class Pending {
  private readonly queues = new Map<RigidBody, Queue>();

  constructor(
    private readonly api: typeof Rapier,
    private readonly prepared: ReadonlyMap<RigidBody, BodyBinding>,
  ) {}

  has(object: RigidBody): boolean {
    return this.queues.has(object);
  }
  /** Whether a queued command alters velocity, so a read needs a preview instead of the authored value. */
  changesVelocity(object: RigidBody): boolean {
    return this.queues.get(object)?.changesVelocity ?? false;
  }
  push(object: RigidBody, command: Command, changesVelocity: boolean): void {
    let queue = this.queues.get(object);
    if (!queue) {
      queue = {
        velocity: authoredVelocity(object),
        changesVelocity: false,
        commands: [],
      };
      this.queues.set(object, queue);
    }
    queue.changesVelocity ||= changesVelocity;
    queue.commands.push(command);
  }
  replay(object: RigidBody, body: Rapier.RigidBody): void {
    const queue = this.queues.get(object);
    if (!queue) return;
    body.setLinvel(queue.velocity.linear, true);
    body.setAngvel(queue.velocity.angular, true);
    for (const command of queue.commands) command(body);
  }
  delete(object: RigidBody): void {
    this.queues.delete(object);
  }
  clear(): void {
    this.queues.clear();
  }

  /** Reads with temporary bodies for the objects not yet prepared; the throwaway world is never stepped. */
  preview<T>(
    objects: Iterable<RigidBody>,
    read: (bodies: ReadonlyMap<RigidBody, BodyBinding>) => T,
  ): T {
    const unprepared = [...objects].filter(
      (object) => !this.prepared.has(object),
    );
    if (!unprepared.length) return read(this.prepared);
    const backend = new this.api.World(new Vector3());
    const bodies = new Map(this.prepared);
    try {
      for (const object of unprepared) {
        object.validate();
        const binding = createBody(this.api, backend, object);
        this.replay(object, binding.body);
        bodies.set(object, binding);
      }
      backend.propagateModifiedBodyPositionsToColliders();
      return read(bodies);
    } finally {
      backend.free();
    }
  }
  previewBody<T>(object: RigidBody, read: (body: Rapier.RigidBody) => T): T {
    return this.preview([object], (bodies) => {
      const binding = bodies.get(object);
      if (!binding) throw new Error("Missing preview body");
      return read(binding.body);
    });
  }
}
