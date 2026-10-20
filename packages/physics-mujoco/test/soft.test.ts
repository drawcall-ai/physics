import { expect, test } from "vitest";
import {
  BoxCollider,
  RigidBody,
  SoftBody,
  SoftAttachment,
  Trigger,
  splitTransform,
} from "@drawcall/physics";
import { PlaneGeometry, Vector3 } from "three";
import { scene, createWorld, steps, earth, ground } from "./fixtures.js";

function cloth(): SoftBody {
  return new SoftBody({
    shape: {
      kind: "surface",
      geometry: new PlaneGeometry(1, 1, 3, 3),
      thickness: 0.01,
    },
    material: {
      density: 1000,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus: 10000,
        poissonRatio: 0.3,
        dampingTime: 0.005,
      },
    },
    selfCollision: true,
  });
}
function solid(): SoftBody {
  return new SoftBody({
    shape: {
      kind: "volume",
      positions: [0, 0, 0, 0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2],
      tetrahedra: [0, 1, 2, 3],
    },
    material: {
      density: 1000,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus: 5000,
        poissonRatio: 0.3,
        dampingTime: 0.01,
      },
    },
  });
}
function coordinate(positions: Float32Array, index: number): number {
  const value = positions[index];
  if (value === undefined) throw new Error("Missing simulation node");
  return value;
}

test("small soft nodes preserve their physical mass", async () => {
  const size = 0.0001;
  const density = 1220;
  const body = new SoftBody({
    shape: {
      kind: "volume",
      positions: [0, 0, 0, size, 0, 0, 0, size, 0, 0, 0, size],
      tetrahedra: [0, 1, 2, 3],
    },
    material: {
      density,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus: 9.8e6,
        poissonRatio: 0.45,
      },
    },
  });
  scene.add(body);
  const world = await createWorld({ fixedDelta: 0.0001 });
  body.applyImpulse(new Vector3((density * size ** 3) / 6, 0, 0));
  steps(world, 1);
  expect(coordinate(body.getPositions(), 0)).toBeCloseTo(world.fixedDelta, 8);
});

test("surface pins carry gravity, rendering deforms and reset restores rest", async () => {
  const body = cloth();
  const pin = new SoftAttachment({
    body,
    region: { min: [-1, 0.49, -1], max: [1, 0.51, 1] },
    target: null,
  });
  scene.add(body, pin);
  const world = await createWorld({ ...earth, fixedDelta: 1 / 240 });
  const rest = body.getPositions();
  steps(world, 100);
  const deformed = body.getPositions();
  expect(coordinate(deformed, 1)).toBeCloseTo(coordinate(rest, 1), 2);
  expect(coordinate(deformed, deformed.length - 2)).toBeLessThan(
    coordinate(rest, rest.length - 2) - 0.001,
  );
  expect(body.geometry.getAttribute("position").getY(15)).toBeCloseTo(
    coordinate(deformed, deformed.length - 2),
    5,
  );
  world.reset();
  expect(Array.from(body.getPositions())).toEqual(Array.from(rest));
});

test("native volume contacts are symmetric and raycasts include deformation", async () => {
  const floor = ground();
  const body = solid();
  body.position.y = 0.1;
  scene.add(body);
  const world = await createWorld({ ...earth, fixedDelta: 1 / 240 });
  steps(world, 200);
  expect(
    body
      .getContacts()
      .some((contact) => contact.otherBody === floor && contact.force > 0),
  ).toBe(true);
  expect(
    floor
      .getContacts()
      .some((contact) => contact.otherBody === body && contact.force > 0),
  ).toBe(true);
  const hit = world.raycast(
    new Vector3(0.04, 1, 0.04),
    new Vector3(0, -1, 0),
    2,
  );
  expect(hit?.kind === "body" && hit.body).toBe(body);
});

test("soft attachment transfers loads to a rigid target and material edits preserve motion", async () => {
  const body = solid();
  const target = new RigidBody({ mass: 1 });
  target.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
  target.position.x = -0.2;
  target.scale.setScalar(2);
  const attachment = new SoftAttachment({ body, region: [0], target });
  scene.add(body, target, attachment);
  const world = await createWorld({ fixedDelta: 1 / 240 });
  for (let i = 0; i < 40; i++) {
    body.applyForce(new Vector3(0, 3, 0));
    steps(world, 1);
  }
  expect(target.getVelocity().linear.y).toBeGreaterThan(0.005);
  const before = body.getPositions();
  const anchor = new Vector3(0.2, 0, 0).applyMatrix4(
    splitTransform(target.matrixWorld).pose,
  );
  expect(new Vector3().fromArray(before).distanceTo(anchor)).toBeLessThan(
    0.002,
  );
  body.setMaterial({
    ...body.material,
    elasticity: {
      model: "saintVenantKirchhoff",
      youngModulus: 7000,
      poissonRatio: 0.3,
      dampingTime: 0.01,
    },
  });
  world.update(0);
  expect(Array.from(body.getPositions())).toEqual(Array.from(before));
  const velocity = target.getVelocity().linear.y;
  expect(velocity).toBeGreaterThan(0.005);
  steps(world, 1);
  expect(coordinate(body.getPositions(), 1)).toBeGreaterThan(
    coordinate(before, 1),
  );
  const movedAnchor = new Vector3(0.2, 0, 0).applyMatrix4(
    splitTransform(target.matrixWorld).pose,
  );
  expect(
    new Vector3().fromArray(body.getPositions()).distanceTo(movedAnchor),
  ).toBeLessThan(0.002);
});

test("curve spring tendons sag, hold their fixed end, and accept impulses", async () => {
  const body = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [0, 1, 0],
        [0.2, 1, 0],
        [0.4, 1, 0],
        [0.6, 1, 0],
      ],
      radius: 0.01,
    },
    material: {
      density: 1000,
      elasticity: {
        model: "spring",
        stretch: { frequency: 20, dampingRatio: 0.8 },
      },
    },
  });
  scene.add(body, new SoftAttachment({ body, region: [0], target: null }));
  const world = await createWorld({ ...earth, fixedDelta: 1 / 240 });
  steps(world, 50);
  expect(coordinate(body.getPositions(), 1)).toBeCloseTo(1, 2);
  expect(coordinate(body.getPositions(), 10)).toBeLessThan(0.9);
  body.applyImpulse(new Vector3(0, 0, 0.01), new Vector3(0.6, 1, 0));
  steps(world, 1);
  expect(coordinate(body.getPositions(), 11)).toBeGreaterThan(0);
});

test("unsupported flex overlap queries fail before a scene is simulated", async () => {
  const trigger = new Trigger();
  trigger.add(new BoxCollider());
  scene.add(cloth(), trigger);
  await expect(createWorld()).rejects.toThrow(/flex overlap queries/);
});

test("a conflicting attachment rolls back without resetting the live soft body", async () => {
  const body = cloth();
  scene.add(body, new SoftAttachment({ body, region: [0], target: null }));
  const world = await createWorld({ ...earth, fixedDelta: 1 / 240 });
  steps(world, 10);
  const before = body.getPositions();
  const duplicate = new SoftAttachment({ body, region: [0], target: null });
  scene.add(duplicate);
  expect(() => world.update(0)).toThrow(/cannot share simulation nodes/);
  duplicate.removeFromParent();
  world.update(0);
  expect(Array.from(body.getPositions())).toEqual(Array.from(before));
  steps(world, 1);
  expect(body.getPositions().every(Number.isFinite)).toBe(true);
});

test("queued forces survive a density rebuild and use the new nodal masses", async () => {
  const changed = solid(),
    control = solid();
  control.position.x = 2;
  const material = { ...control.material, density: 2000, staticFriction: 0.8 };
  control.setMaterial(material);
  scene.add(changed, control);
  const world = await createWorld({ fixedDelta: 1 / 240 });
  const force = new Vector3(0, 10, 0);
  changed.applyForce(force);
  control.applyForce(force);
  changed.setMaterial(material);
  steps(world, 1);
  const actual = changed.getPositions(),
    expected = control.getPositions();
  expect(coordinate(actual, 1)).toBeGreaterThan(0);
  for (let i = 1; i < actual.length; i += 3)
    expect(coordinate(actual, i)).toBeCloseTo(coordinate(expected, i), 7);
});
