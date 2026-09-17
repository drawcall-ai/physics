import * as THREE from "three";
import {
  BoxCollider,
  CapsuleCollider,
  CylinderCollider,
  MeshCollider,
  RigidBody,
  resolveCollider,
  splitTransform,
  SphereCollider,
  type PhysicsWorld,
  type AutoColliders,
} from "@drawcall/physics";

import type { Case } from "./cases";

export function specimen(world: PhysicsWorld, spec: Case, spin = false) {
  const root = new THREE.Group();
  const geometries: THREE.BufferGeometry[] = [];
  const materials: THREE.Material[] = [];
  const bodies: RigidBody[] = [];
  function mesh(geometry: THREE.BufferGeometry, color: string) {
    const material = new THREE.MeshStandardMaterial({ color });
    geometries.push(geometry);
    materials.push(material);
    return new THREE.Mesh(geometry, material);
  }
  function body(
    type: "dynamic" | "static" | "kinematic",
    visual: THREE.Mesh,
    colliders: AutoColliders = "auto",
  ) {
    const body = new RigidBody({
      world,
      type,
      colliders,
      ...(type === "static" ? {} : { mass: 2 }),
    });
    if (spin && type === "dynamic")
      body.setVelocity({ angular: new THREE.Vector3(1.4, 0.7, 1.1) });
    bodies.push(body);
    body.add(visual);
    return body;
  }
  const floor = body(
    "static",
    mesh(new THREE.BoxGeometry(10, 0.2, 10), "#526478"),
  );
  floor.position.y = -0.1;
  root.add(floor);
  const kind = spec.geometry ?? spec.kind;
  const geometry =
    kind === "sphere"
      ? new THREE.SphereGeometry(0.5, 32, 24)
      : kind === "capsule"
        ? new THREE.CapsuleGeometry(0.4, 1, 12, 24)
        : kind === "cylinder"
          ? new THREE.CylinderGeometry(0.5, 0.5, 1, 32)
          : spec.kind === "triangle mesh" || spec.compound
            ? new THREE.BoxGeometry(1.4, 0.35, 1.4)
            : new THREE.BoxGeometry();
  const visual = mesh(geometry, "#eaa65a");
  const platform =
    spec.kind === "triangle mesh" ||
    spec.type === "kinematic" ||
    spec.type === "static";
  const target = body(
    spec.type ?? (spec.kind === "triangle mesh" ? "static" : "dynamic"),
    visual,
    spec.kind === "convex hull"
      ? "convexHull"
      : spec.kind === "triangle mesh"
        ? "trimesh"
        : "auto",
  );
  target.name = spec.name;
  let collider;
  if (spec.explicit) {
    collider =
      spec.kind === "box"
        ? new BoxCollider()
        : spec.kind === "sphere"
          ? new SphereCollider()
          : spec.kind === "capsule"
            ? new CapsuleCollider({ radius: 0.4 })
            : spec.kind === "cylinder"
              ? new CylinderCollider()
              : new MeshCollider({
                  approximation:
                    spec.kind === "triangle mesh" ? "trimesh" : "convexHull",
                });
    if (collider instanceof MeshCollider) collider.setGeometry(geometry);
    target.add(collider);
  }
  const parent = new THREE.Group().add(target);
  root.add(parent);
  const scale = new THREE.Vector3(...spec.scale);
  if (spec.placement === "ancestor") parent.scale.copy(scale);
  if (spec.placement === "body") target.scale.copy(scale);
  if (spec.placement === "collider" || spec.placement === "combined") {
    visual.scale.copy(scale);
    collider?.scale.copy(scale);
    // Nonzero offsets prove that scale affects positions as well as dimensions.
    visual.position.x = 0.25;
    if (collider) collider.position.x = 0.25;
  }
  if (spec.placement === "combined") {
    parent.scale.setScalar(1.5);
    target.scale.setScalar(0.75);
    parent.rotation.y = 0.35;
  }
  if (spec.shear) {
    visual.matrixAutoUpdate = false;
    visual.matrix.makeShear(0.5, 0, 0, 0, 0, 0);
    if (collider) {
      collider.matrixAutoUpdate = false;
      collider.matrix.copy(visual.matrix);
    }
  }
  const surfaces = [visual];
  if ((spec.kind === "triangle mesh" && !spec.explicit) || spec.compound) {
    for (const side of [-1, 1]) {
      const step = mesh(new THREE.BoxGeometry(1.4, 0.35, 1.4), "#eaa65a");
      step.position.set(side * 2, side * 0.45, 0);
      // Quarter turns remain shear-free beneath nonuniform body scale.
      step.rotation.y = (side * Math.PI) / 2;
      step.scale.set(0.8, 1.5, 1.2);
      target.add(step);
      surfaces.push(step);
    }
  }
  parent.position.y = platform ? 1.5 : 5;
  root.updateMatrixWorld(true);
  const contacts = platform
    ? surfaces.map((surface) => {
        const bounds = new THREE.Box3().setFromObject(surface);
        const falling = mesh(new THREE.BoxGeometry(0.5, 0.5, 0.5), "#83d9cb");
        const probe = body("dynamic", falling);
        bounds.getCenter(probe.position);
        probe.position.y = bounds.max.y + 3;
        root.add(probe);
        return {
          falling,
          top: () => new THREE.Box3().setFromObject(surface).max.y,
        };
      })
    : [{ falling: target, top: () => 0 }];
  let time = 0;
  let initial: THREE.Matrix4 | undefined;
  const stop =
    spec.type === "kinematic"
      ? world.onAfterStep((delta) => {
          initial ??= splitTransform(target.matrixWorld).pose;
          time += delta;
          const position = new THREE.Vector3().setFromMatrixPosition(initial);
          position.y += 0.65 * Math.sin(time * 1.3);
          target.setKinematicTarget(initial.clone().setPosition(position));
        })
      : undefined;
  return {
    root,
    target,
    boundsError() {
      return Math.max(
        ...target.getColliders().map((collider) => {
          const { shape, matrix } = resolveCollider(target, collider);
          const geometry =
            shape.kind === "box"
              ? new THREE.BoxGeometry(...shape.size)
              : shape.kind === "sphere"
                ? new THREE.SphereGeometry(shape.radius, 32, 24)
                : shape.kind === "capsule"
                  ? new THREE.CapsuleGeometry(
                      shape.radius,
                      shape.height,
                      12,
                      24,
                    )
                  : shape.kind === "cylinder"
                    ? new THREE.CylinderGeometry(
                        shape.radius,
                        shape.radius,
                        shape.height,
                        32,
                      )
                    : shape.geometry;
          const collision = new THREE.Mesh(geometry, visual.material);
          collision.matrixAutoUpdate = false;
          collision.matrix
            .copy(splitTransform(target.matrixWorld).pose)
            .multiply(matrix);
          collision.updateMatrixWorld(true);
          const actual = new THREE.Box3().setFromObject(collision, true);
          const source =
            collider.source instanceof THREE.Mesh ? collider.source : visual;
          const expected = new THREE.Box3().setFromObject(source, true);
          geometry.dispose();
          return Math.max(
            actual.min.distanceTo(expected.min),
            actual.max.distanceTo(expected.max),
          );
        }),
      );
    },
    gap() {
      root.updateMatrixWorld(true);
      return contacts.reduce((worst, { falling, top }) => {
        const gap = new THREE.Box3().setFromObject(falling, true).min.y - top();
        return Math.abs(gap) > Math.abs(worst) ? gap : worst;
      }, 0);
    },
    dispose() {
      stop?.();
      for (const body of bodies) body.dispose();
      root.removeFromParent();
      geometries.forEach((g) => g.dispose());
      materials.forEach((m) => m.dispose());
    },
  };
}

export function verify(world: PhysicsWorld, spec: Case) {
  const item = specimen(world, spec);
  try {
    try {
      world.update(world.fixedDelta);
      if (!spec.error && item.boundsError() > 1e-5)
        throw new Error("Collider bounds differ from visual geometry");
      if (spec.edit) item.target.scale.setScalar(2);
      for (let i = 0; i < 239; i++) world.update(world.fixedDelta);
    } catch (error) {
      if (
        spec.error &&
        error instanceof Error &&
        error.message.includes(spec.error)
      )
        return `PASS: rejected (${spec.error})`;
      throw error;
    }
    if (spec.error) throw new Error(`Expected rejection: ${spec.error}`);
    const gap = item.gap();
    if (Math.abs(gap) > 0.06)
      throw new Error(`Contact gap ${gap.toFixed(3)} m`);
    return `PASS: bounds match; contact gap ${gap.toFixed(3)} m`;
  } finally {
    item.dispose();
  }
}
