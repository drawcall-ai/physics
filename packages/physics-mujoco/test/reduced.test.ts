import loadMujoco from "@mujoco/mujoco";
import { expect, test } from "vitest";
import {
  type ReducedModel,
  RigidBody,
  SoftAttachment,
  SoftBody,
  SphereCollider,
  BoxCollider,
  PrismaticJoint,
} from "@drawcall/physics";
import { evaluateReduced } from "../src/reduced/energy.js";
import { Matrix4, Vector3 } from "three";
import { compile } from "../src/model/compile.js";
import { reducedMesh } from "../src/reduced/model.js";
import { array, at } from "../src/heap.js";
import { value } from "../src/reduced/matrix.js";
import { scene, createWorld, steps, earth } from "./fixtures.js";

function data(
  law: "linear" | "neoHookean" | "saintVenantKirchhoff" = "linear",
): ReducedModel {
  const positions = [0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0];
  const basis = Array.from({ length: 36 }, (_, i) =>
    i >= 27 && (i - 27) % 4 === 0 ? 1 : 0,
  );
  const mass = Array.from({ length: 36 }, (_, i) => {
    const row = Math.floor(i / 6),
      column = i % 6;
    if (row === column) return row < 3 ? 4 : 1;
    if (Math.abs(row - column) === 3) return 1;
    if (row >= 3 && column >= 3) return 0.1;
    return 0;
  });
  const mu = 1000 / 2.6,
    lambda = (1000 * 0.3) / 1.3 / 0.4;
  return {
    dofs: 3,
    material: {
      density: 1000,
      elasticity: {
        model: law,
        youngModulus: 1000,
        poissonRatio: 0.3,
        dampingTime: 0.1,
      },
    },
    positions,
    triangles: [0, 3, 1, 1, 3, 2, 0, 2, 3, 0, 1, 2],
    mount: [0, 1, 2],
    coordinates: [3],
    basis,
    mass,
    stiffness:
      law === "linear"
        ? [200, -30, 0, -30, 300, -20, 0, -20, 100]
        : [mu / 6, 0, 0, 0, (lambda + 2 * mu) / 6, 0, 0, 0, mu / 6],
    quadrature: {
      weights: [1 / 6],
      gradients: Array.from({ length: 27 }, (_, i) =>
        Math.floor(i / 3) === 1 + 3 * (i % 3) ? 1 : 0,
      ),
    },
    gravity: [4, 0, 0, 0, 4, 0, 0, 0, 4, 1, 0, 0, 0, 1, 0, 0, 0, 1],
    contactRegion: { min: [-1, -1, -1], max: [2, 2, 2] },
    mountRegion: { min: [-1, -0.01, -1], max: [2, 0.01, 2] },
    renderPositions: positions,
    renderTriangles: [0, 3, 1, 1, 3, 2, 0, 2, 3, 0, 1, 2],
    renderBasis: basis,
  };
}

test.each(["neoHookean", "saintVenantKirchhoff"] as const)(
  "differentiates %s energy consistently",
  (law) => {
    const model = data(law);
    const q = [0.08, -0.04, 0.03],
      epsilon = 1e-6;
    const response = evaluateReduced(model, q);
    for (let column = 0; column < model.dofs; column++) {
      const a = evaluateReduced(
        model,
        q.map((v, i) => v + (i === column ? epsilon : 0)),
      );
      const b = evaluateReduced(
        model,
        q.map((v, i) => v - (i === column ? epsilon : 0)),
      );
      expect((a.energy - b.energy) / (2 * epsilon)).toBeCloseTo(
        -value(response.force, column),
        6,
      );
      for (let row = 0; row < model.dofs; row++)
        expect(
          -(value(a.force, row) - value(b.force, row)) / (2 * epsilon),
        ).toBeCloseTo(value(response.stiffness, row * model.dofs + column), 6);
    }
    evaluateReduced(model, [0, 0, 0]).stiffness.forEach((v, i) =>
      expect(v).toBeCloseTo(value(model.stiffness, i), 8),
    );
    expect(() => evaluateReduced(model, [0, -1.1, 0])).toThrow(/inverted/);
  },
);

test("fixed tendon armature preserves dense off-axis reduced mass", async () => {
  const api = await loadMujoco();
  const model = data();
  const body = new SoftBody({ model });
  const pin = new SoftAttachment({ body, region: model.mount, target: null });
  const sim = compile(
    api,
    [],
    new Map(),
    [],
    {
      fixedDelta: 0.01,
      gravity: [0, 0, 0],
      solverIterations: 100,
      frictionImpedanceRatio: 1,
      frictionCone: "elliptic",
    },
    new Map(),
    [],
    new Map([[body, reducedMesh(body)]]),
    [pin],
  );
  const matrix = new api.DoubleBuffer(sim.model.nv ** 2);
  try {
    api.mj_fullM(sim.model, sim.data, matrix);
    for (let i = 0; i < model.mass.length; i++)
      expect(at(matrix.GetView(), i)).toBeCloseTo(model.mass[i] ?? NaN, 10);
    const q = [0.01, -0.1, 0.03];
    array(sim.data.qpos).set([0, 0, 0, ...q]);
    api.mj_forward(sim.model, sim.data);
    const response = evaluateReduced(model, q);
    for (let i = 0; i < 3; i++)
      expect(at(sim.data.qfrc_passive, i + 3)).toBeCloseTo(
        value(response.force, i),
        10,
      );
  } finally {
    matrix.delete();
    sim.free();
  }
});

test("linear static force balance, material rebuild and reset retain the reduced state", async () => {
  const model = data();
  const body = new SoftBody({ model });
  scene.add(
    body,
    new SoftAttachment({ body, region: model.mount, target: null }),
  );
  const world = await createWorld();
  for (let i = 0; i < 300; i++) {
    body.applyForce(new Vector3(0, -1, 0), new Vector3(0, 1, 0));
    steps(world, 1);
  }
  const points = body.getPositions();
  const q = [
    value(points, 9) - value(points, 0),
    value(points, 10) - value(points, 1) - 1,
    value(points, 11) - value(points, 2),
  ];
  const response = evaluateReduced(model, q);
  expect(response.force[0]).toBeCloseTo(0, 4);
  expect(response.force[1]).toBeCloseTo(1, 4);
  expect(response.force[2]).toBeCloseTo(0, 4);
  expect(body.geometry.getAttribute("position").getY(3)).toBeCloseTo(
    value(points, 10),
    6,
  );
  body.setMaterial({ ...body.material, staticFriction: 0.7 });
  world.update(0);
  expect(Array.from(body.getPositions())).toEqual(Array.from(points));
  world.reset();
  expect(Array.from(body.getPositions())).toEqual(data().positions);
});

test("a translating kinematic mount moves the reduced component and rejects rotation", async () => {
  const model = data();
  const body = new SoftBody({ model });
  const target = new RigidBody({ bodyType: "kinematic" });
  scene.add(
    body,
    target,
    new SoftAttachment({ body, region: model.mount, target }),
  );
  const world = await createWorld();
  target.setKinematicTarget(new Matrix4().makeTranslation(0.1, 0, 0));
  steps(world, 100);
  expect(value(body.getPositions(), 0)).toBeCloseTo(0.1, 3);
  body.setLinearDamping(0.1);
  world.update(0);
  steps(world, 100);
  expect(value(body.getPositions(), 0)).toBeCloseTo(0.1, 3);
  expect(() =>
    target.setKinematicTarget(
      new Matrix4().makeRotationY(0.1).setPosition(0.1, 0, 0),
    ),
  ).toThrow(/rotating mounts/);
});

test.each(["neoHookean", "saintVenantKirchhoff"] as const)(
  "%s forces and tangent settle at the nonlinear analytical equilibrium",
  async (law) => {
    const model = data(law);
    const body = new SoftBody({ model });
    scene.add(
      body,
      new SoftAttachment({ body, region: model.mount, target: null }),
    );
    const world = await createWorld();
    const target = [0.02, -0.1, 0.01];
    const applied = new Vector3()
      .fromArray(evaluateReduced(model, target).force)
      .negate();
    for (let i = 0; i < 600; i++) {
      body.applyForce(applied, new Vector3(0, 1, 0));
      steps(world, 1);
    }
    const points = body.getPositions();
    for (let axis = 0; axis < 3; axis++)
      expect(
        value(points, 9 + axis) - value(points, axis) - (axis === 1 ? 1 : 0),
      ).toBeCloseTo(value(target, axis), 4);
  },
);

test("a rigid sphere's weight deforms the reduced body with equal and opposite contact forces", async () => {
  const model: ReducedModel = {
    ...data(),
    stiffness: [1000, 0, 0, 0, 1000, 0, 0, 0, 1000],
  };
  const body = new SoftBody({ model });
  const sphere = new RigidBody({ mass: 0.1 });
  sphere.add(new SphereCollider({ radius: 0.05 }));
  sphere.position.set(0, 1.07, 0);
  const guide = new PrismaticJoint({ body0: null, body1: sphere, axis: "Y" });
  scene.add(
    body,
    sphere,
    guide,
    new SoftAttachment({ body, region: model.mount, target: null }),
  );
  const world = await createWorld({ ...earth, fixedDelta: 0.005 });
  steps(world, 600);
  const bodyForce = body
    .getContacts()
    .reduce((sum, contact) => sum + contact.normal.y * contact.force, 0);
  const sphereForce = sphere
    .getContacts()
    .reduce((sum, contact) => sum + contact.normal.y * contact.force, 0);
  expect(bodyForce).toBeCloseTo(-0.981, 2);
  expect(sphereForce).toBeCloseTo(0.981, 2);
  const points = body.getPositions();
  expect(value(points, 10) - value(points, 1) - 1).toBeCloseTo(
    -(9.81 + 0.981) / 1000,
    4,
  );
  expect(sphere.getVelocity().linear.length()).toBeLessThan(1e-4);
});

test("uncovered surface intersections with static geometry fail before solving", async () => {
  const model: ReducedModel = {
    ...data(),
    triangles: [0, 1, 2],
    contactRegion: { min: [-1, -0.01, -1], max: [2, 0.01, 2] },
  };
  const body = new SoftBody({ model });
  const obstacle = new RigidBody({ bodyType: "static" });
  obstacle.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
  obstacle.position.set(0.2, 0.4, -0.02);
  scene.add(
    body,
    obstacle,
    new SoftAttachment({ body, region: model.mount, target: null }),
  );
  await expect(createWorld().then(() => "built")).rejects.toThrow(
    /outside the baked contact region/,
  );
});

test("the uncovered-surface guard follows reduced deformation", async () => {
  const model: ReducedModel = {
    ...data(),
    triangles: [0, 1, 2],
    contactRegion: { min: [-1, -0.01, -1], max: [2, 0.01, 2] },
  };
  const body = new SoftBody({ model });
  const obstacle = new RigidBody({ bodyType: "static" });
  obstacle.add(new BoxCollider({ size: [0.04, 0.1, 0.04] }));
  obstacle.position.set(0.12, 1.05, 0);
  scene.add(
    body,
    obstacle,
    new SoftAttachment({ body, region: model.mount, target: null }),
  );
  const world = await createWorld();
  expect(() => {
    for (let i = 0; i < 100; i++) {
      body.applyForce(
        new Vector3(30, 0, 0),
        new Vector3().fromArray(body.getPositions(), 9),
      );
      steps(world, 1);
    }
  }).toThrow(/outside the baked contact region/);
});

test("the baked mounting region may touch its static attachment target", async () => {
  const model: ReducedModel = {
    ...data(),
    triangles: [0, 1, 2],
    contactRegion: { min: [-0.01, 0.99, -0.01], max: [0.01, 1.01, 0.01] },
  };
  const body = new SoftBody({ model });
  const target = new RigidBody({ bodyType: "static" });
  target.add(new BoxCollider({ size: [2, 0.1, 2] }));
  target.position.set(0.5, -0.05, 0.5);
  scene.add(
    body,
    target,
    new SoftAttachment({ body, region: model.mount, target }),
  );
  const world = await createWorld();
  steps(world, 5);
  for (const [i, coordinate] of body.getPositions().entries())
    expect(coordinate).toBeCloseTo(value(model.positions, i), 9);
  target.rotation.y = 0.1;
  expect(() => world.update(0)).toThrow(/rotating mounts/);
  target.rotation.y = 0;
  world.reset();
});

test("a box pressing a contact triangle's interior transfers its load to the reduced coordinates", async () => {
  const model: ReducedModel = {
    ...data(),
    stiffness: [1000, 0, 0, 0, 1000, 0, 0, 0, 1000],
  };
  const body = new SoftBody({ model });
  const box = new RigidBody({ mass: 0.1 });
  box.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
  box.position.set(0.2, 0.4, -0.07);
  const guide = new PrismaticJoint({ body0: null, body1: box, axis: "Z" });
  scene.add(
    body,
    box,
    guide,
    new SoftAttachment({ body, region: model.mount, target: null }),
  );
  const world = await createWorld({ fixedDelta: 0.005 });
  for (let i = 0; i < 600; i++) {
    box.applyForce(new Vector3(0, 0, 1));
    steps(world, 1);
  }
  const bodyForce = body
    .getContacts()
    .reduce((sum, contact) => sum + contact.normal.z * contact.force, 0);
  const boxForce = box
    .getContacts()
    .reduce((sum, contact) => sum + contact.normal.z * contact.force, 0);
  expect(bodyForce).toBeCloseTo(1, 2);
  expect(boxForce).toBeCloseTo(-1, 2);
  // Native flex contacts use inverse-distance weights, rather than FEM barycentric weights.
  expect(value(body.getPositions(), 11)).toBeGreaterThan(0);
  expect(value(body.getPositions(), 11)).toBeLessThan(1 / 1000);
  expect(box.getVelocity().linear.length()).toBeLessThan(1e-4);
});
