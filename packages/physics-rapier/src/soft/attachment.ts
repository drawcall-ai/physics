import type * as Rapier from "@dimforge/rapier3d-compat";
import type { SoftAttachment } from "@drawcall/physics";
import { selectSoftNodes } from "@drawcall/physics/backend";
import { Vector3 } from "three";
import { bodyPose } from "../rigid/body.js";
import type { SoftBinding } from "./body.js";

export class SoftMount {
  private readonly nodes: number[];
  private enabled = false;
  private native: Rapier.SoftBody;
  private readonly anchors: Vector3[];

  constructor(
    readonly object: SoftAttachment,
    private readonly body: SoftBinding,
    private readonly target: Rapier.RigidBody | null,
  ) {
    this.nodes = selectSoftNodes(object, body.rest);
    this.native = body.native;
    const inverse = target && bodyPose(target).invert();
    this.anchors = this.nodes.map((node) => {
      const point = new Vector3().copy(body.native.particlePosition(node));
      return inverse ? point.applyMatrix4(inverse) : point;
    });
  }

  prepare(): void {
    if (this.native !== this.body.native) {
      this.native = this.body.native;
      this.enabled = false;
    }
    if (this.enabled === this.object.enabled) return;
    if (!this.object.enabled) return this.release();
    const body = this.body.native;
    const attached = new Set(
      Array.from({ length: body.numAttachments() }, (_, i) =>
        body.attachmentParticle(i),
      ),
    );
    if (
      this.nodes.some(
        (node) => body.isParticlePinned(node) || attached.has(node),
      )
    )
      throw new Error("Soft attachments cannot share simulation nodes");
    for (const [index, node] of this.nodes.entries()) {
      const anchor = this.anchors[index];
      if (!anchor) throw new Error("Missing soft attachment anchor");
      if (!this.target) {
        body.setParticlePinned(node, true);
        body.setParticleKinematicTarget(node, anchor);
        continue;
      }
      const position = body.particlePosition(node);
      const velocity = body.particleVelocity(node);
      // Rapier captures an attachment's anchor from the current particle position.
      body.setParticlePosition(
        node,
        anchor.clone().applyMatrix4(bodyPose(this.target)),
      );
      body.attachParticle(node, this.target);
      body.setParticlePosition(node, position);
      body.setParticleVelocity(node, velocity);
    }
    this.enabled = true;
  }

  release(): void {
    if (!this.enabled) return;
    for (const node of this.nodes) {
      if (this.target) this.body.native.detachParticle(node);
      else this.body.native.setParticlePinned(node, false);
    }
    this.enabled = false;
  }
}
