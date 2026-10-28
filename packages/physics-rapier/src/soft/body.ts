import type * as Rapier from "@dimforge/rapier3d-compat";
import { splitTransform, type SoftBody } from "@drawcall/physics";
import { lockScale, updateSoftGeometry } from "@drawcall/physics/backend";
import { Vector3, type Matrix4 } from "three";
import { addVolume, point, shape, transformPositions } from "./shape.js";

export class SoftBinding {
  native: Rapier.SoftBody;
  readonly rest: Float32Array;
  private readonly initial: Float32Array;
  private topology;
  private readonly transform: Matrix4;
  private readonly scale: Vector3;
  private version: number;
  private density: number;
  private damping: number;
  private readonly volumes: Float32Array;
  private model: string;
  private readonly forces: { force: Vector3; node: number | undefined }[] = [];

  constructor(
    private readonly api: typeof Rapier,
    private readonly simulation: Rapier.World,
    readonly object: SoftBody,
  ) {
    object.validate();
    this.transform = object.matrixWorld.clone();
    this.scale = splitTransform(object.matrixWorld).scale;
    validateMaterial(object);
    this.topology = shape(api, object, this.transform);
    if (!(this.topology.volume > 0))
      throw new Error("Soft body must have positive physical volume");
    this.volumes = this.topology.volumes ?? this.cageVolumes();
    this.configure();
    this.native = simulation.createSoftBody(this.topology.desc);
    this.initial = this.native.particlePositions();
    this.rest = this.initial.slice();
    transformPositions(this.rest, object.matrixWorld.clone().invert());
    this.version = object.version;
    this.density = object.material.density;
    this.damping = object.linearDamping;
    this.model = model(object);
  }

  prepare(): void {
    this.object.validate();
    lockScale(
      this.object.name || this.object.type,
      this.scale,
      splitTransform(this.object.matrixWorld).scale,
    );
    if (this.version === this.object.version) return;
    validateMaterial(this.object);
    const nextModel = model(this.object);
    if (nextModel !== this.model)
      this.topology = shape(this.api, this.object, this.transform);
    this.configure();
    const { density } = this.object.material;
    if (
      density !== this.density ||
      nextModel !== this.model ||
      this.damping !== this.object.linearDamping
    )
      this.replace(true);
    else {
      this.native.setMaterial(material(this.api, this.object));
      this.updateContact();
      this.native.wakeUp();
    }
    this.density = density;
    this.damping = this.object.linearDamping;
    this.model = nextModel;
    this.version = this.object.version;
  }

  writeBack(): void {
    const positions = this.topology.skin
      ? this.native.meshVertices(0)
      : this.native.particlePositions();
    if (!positions.every(Number.isFinite))
      throw new Error("Rapier produced non-finite soft-body positions");
    const vertices = this.topology.vertices;
    if (!vertices) return updateSoftGeometry(this.object, positions);
    const render = new Float32Array(vertices.length * 3);
    vertices.forEach((index, vertex) =>
      point(positions, index).toArray(render, vertex * 3),
    );
    updateSoftGeometry(this.object, render);
  }

  restore(): void {
    this.forces.length = 0;
    this.replace(false);
    this.writeBack();
  }

  force(force: Vector3, at?: Vector3): void {
    this.forces.push({
      force: force.clone(),
      node: at ? this.closest(at) : undefined,
    });
  }

  applyForces(): void {
    for (const { force, node } of this.forces.splice(0)) {
      if (node !== undefined) this.native.addParticleForce(node, force, true);
      else
        this.distribute(force, (node, load) =>
          this.native.addParticleForce(node, load, true),
        );
    }
  }

  impulse(impulse: Vector3, at?: Vector3): void {
    if (at) this.native.applyParticleImpulse(this.closest(at), impulse, true);
    else
      this.distribute(impulse, (node, load) =>
        this.native.applyParticleImpulse(node, load, true),
      );
  }

  private distribute(
    load: Vector3,
    apply: (node: number, load: Vector3) => void,
  ): void {
    const mass = this.native.mass();
    const portion = new Vector3();
    for (let node = 0; node < this.native.numParticles(); node++)
      apply(
        node,
        portion
          .copy(load)
          .multiplyScalar(this.native.particleMass(node) / mass),
      );
  }

  private configure(): void {
    const { desc } = this.topology;
    const { elasticity, density } = this.object.material;
    desc
      .setMasses(this.volumes.map((volume) => volume * density))
      .setLinearDamping(this.object.linearDamping)
      .setMaterial(material(this.api, this.object))
      .setSelfContacts(
        "shape" in this.object.options &&
          (this.object.options.selfCollision ?? false),
      )
      .setSurfaceCollider(
        this.api.ColliderDesc.ball(0.01)
          .setFriction(friction(this.object))
          .setRestitution(this.object.material.restitution ?? 0),
      );
    if (elasticity.model === "corotational")
      desc.setCellModel(this.api.SoftBodyCellModel.Corotational);
    if (elasticity.model === "neoHookean")
      desc.setCellModel(this.api.SoftBodyCellModel.NeoHookean);
  }

  private cageVolumes(): Float32Array {
    // The native mesher owns the cage topology. Read its cell measures before assigning masses.
    const cage = this.simulation.createSoftBody(this.topology.desc);
    try {
      const cells = cage.cells();
      const volumes = new Float32Array(cage.numParticles());
      cells.forEach((node, index) => {
        addVolume(
          volumes,
          node,
          cage.cellRestVolume(Math.floor(index / 4)) / 4,
        );
      });
      const total = volumes.reduce((sum, volume) => sum + volume, 0);
      if (!(total > 0))
        throw new Error("Rapier volume cage has no physical volume");
      // The cage encloses the surface; preserve the authored body's total mass.
      return volumes.map((volume) => (volume * this.topology.volume) / total);
    } finally {
      this.simulation.removeSoftBody(cage);
    }
  }

  private updateContact(): void {
    for (let i = 0; i < this.native.numMeshes(); i++) {
      const collider = this.native.meshCollider(i);
      if (!collider)
        throw new Error("Rapier soft collision mesh has no collider");
      collider.setFriction(friction(this.object));
      collider.setRestitution(this.object.material.restitution ?? 0);
    }
  }

  private replace(preserve: boolean): void {
    const previous = this.native;
    const positions = preserve ? previous.particlePositions() : this.initial;
    const velocities = preserve
      ? previous.particleVelocities()
      : new Float32Array(positions.length);
    const next = this.simulation.createSoftBody(this.topology.desc);
    for (let i = 0; i < next.numParticles(); i++) {
      next.setParticlePosition(i, point(positions, i));
      next.setParticleVelocity(i, point(velocities, i));
    }
    this.native = next;
    this.simulation.removeSoftBody(previous);
  }

  private closest(at: Vector3): number {
    let selected = 0,
      distance = Infinity;
    const positions = this.native.particlePositions();
    for (let i = 0; i < this.native.numParticles(); i++) {
      const current = point(positions, i).distanceToSquared(at);
      if (current < distance) {
        selected = i;
        distance = current;
      }
    }
    return selected;
  }
}

function material(api: typeof Rapier, body: SoftBody): Rapier.SoftBodyMaterial {
  const result = new api.SoftBodyMaterial();
  const elasticity = body.material.elasticity;
  if (elasticity.model === "spring") {
    result.edgeSoftness = {
      naturalFrequency: elasticity.stretch.frequency,
      dampingRatio: elasticity.stretch.dampingRatio,
    };
    result.bendSoftness = elasticity.bend
      ? {
          naturalFrequency: elasticity.bend.frequency,
          dampingRatio: elasticity.bend.dampingRatio,
        }
      : { naturalFrequency: 0, dampingRatio: 0 };
  } else if (
    elasticity.model === "corotational" ||
    elasticity.model === "neoHookean"
  ) {
    result.youngModulus = elasticity.youngModulus;
    result.poissonRatio = elasticity.poissonRatio;
    result.elasticDampingRatio = elasticity.dampingRatio ?? 0;
  }
  return result;
}

/** A disabled spring has no native constraint; enabling it requires rebuilding from rest. */
function model(body: SoftBody): string {
  const elasticity = body.material.elasticity;
  if (elasticity.model !== "spring") return elasticity.model;
  return `spring:${elasticity.stretch.frequency > 0}:${(elasticity.bend?.frequency ?? 0) > 0}`;
}

function friction(body: SoftBody): number {
  return body.material.dynamicFriction ?? body.material.staticFriction ?? 0.5;
}

function validateMaterial(body: SoftBody): void {
  if (!("shape" in body.options))
    throw new Error("Rapier does not support reduced FEM models");
  const {
    elasticity,
    staticFriction,
    dynamicFriction,
    rollingFriction,
    torsionalFriction,
  } = body.material;
  if (elasticity.model === "linear")
    throw new Error("Rapier does not support linear elasticity");
  if (body.options.shape.kind === "volume" && elasticity.model === "spring")
    throw new Error("Rapier volumes require solid elasticity");
  if (body.options.shape.kind !== "volume" && elasticity.model !== "spring")
    throw new Error("Rapier surfaces and curves require spring elasticity");
  if (elasticity.model === "saintVenantKirchhoff")
    throw new Error(
      "Rapier does not support Saint Venant–Kirchhoff elasticity",
    );
  if (elasticity.model !== "spring" && elasticity.poissonRatio > 0.499)
    throw new Error("Rapier supports Poisson ratios up to 0.499");
  if (elasticity.model !== "spring" && elasticity.dampingTime)
    throw new Error("Rapier does not support dampingTime; use dampingRatio");
  if (
    staticFriction !== undefined &&
    dynamicFriction !== undefined &&
    staticFriction !== dynamicFriction
  )
    throw new Error("Rapier requires equal static and dynamic friction");
  if (rollingFriction || torsionalFriction)
    throw new Error("Rapier colliders have no torsional or rolling friction");
}
