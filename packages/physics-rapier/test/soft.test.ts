import { expect, it } from "vitest";
import { BoxGeometry, Group, Matrix4, PlaneGeometry, Vector3 } from "three";
import { SoftAttachment, SoftBody, type SoftMaterial } from "@drawcall/physics";
import { box, createWorld, earth, inertialBody, steps } from "./fixtures.js";

const fabric: SoftMaterial = {
  density: 500,
  elasticity: {
    model: "spring",
    stretch: { frequency: 30, dampingRatio: 1 },
    bend: { frequency: 1, dampingRatio: 1 },
  },
};
const rubber: SoftMaterial = {
  density: 100,
  elasticity: {
    model: "corotational",
    youngModulus: 2000,
    poissonRatio: 0.35,
    dampingRatio: 0.5,
  },
};

it("omits disabled springs and preserves state when enabling them later", async () => {
  const world = await createWorld();
  const free: SoftMaterial = {
    density: 3 / (Math.PI * 0.1 ** 2),
    elasticity: { model: "spring", stretch: { frequency: 0, dampingRatio: 0 } },
  };
  const rope = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [0, 0, 0],
        [0.5, 0, 0],
        [1, 0, 0],
      ],
      radius: 0.1,
    },
    material: free,
  });
  world.scene.add(
    rope,
    new SoftAttachment({ body: rope, region: [0], target: null }),
  );
  rope.applyImpulse(new Vector3(0, 1, 0), new Vector3(1, 0, 0));
  steps(world, 100);
  const released = rope.getPositions();
  expect(released[4]).toBeCloseTo(0, 5);
  // The endpoint owns half its adjacent segment: 0.75 kg, so 1 N·s gives 4/3 m/s.
  expect(released[7]).toBeCloseTo(4 / 3, 4);
  rope.setMaterial({ ...fabric, density: free.density });
  world.update(0);
  expect(rope.getPositions()).toEqual(released);
  steps(world, 100);
  expect(rope.getPositions().every(Number.isFinite)).toBe(true);
  const bent = rope.getPositions();
  rope.setMaterial(free);
  world.update(0);
  expect(rope.getPositions()).toEqual(bent);
  steps(world, 10);
  expect(rope.getPositions().every(Number.isFinite)).toBe(true);
  expect(rope.getPositions()[0]).toBeCloseTo(0, 5);
});

it("simulates cloth and ropes without bending resistance", async () => {
  const world = await createWorld(earth);
  const material: SoftMaterial = {
    density: 1000,
    elasticity: {
      model: "spring",
      stretch: { frequency: 30, dampingRatio: 1 },
    },
  };
  const cloth = new SoftBody({
    shape: {
      kind: "surface",
      geometry: new PlaneGeometry(1, 1, 2, 2).rotateX(-Math.PI / 2),
      thickness: 0.01,
    },
    material,
  });
  const rope = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [2, 0, 0],
        [2.5, 0, 0],
        [3, 0, 0],
      ],
      radius: 0.1,
    },
    material,
  });
  world.scene.add(
    cloth,
    rope,
    new SoftAttachment({ body: cloth, region: [0, 1, 2], target: null }),
    new SoftAttachment({ body: rope, region: [0], target: null }),
  );
  steps(world, 60);
  for (const body of [cloth, rope]) {
    expect(body.getPositions().every(Number.isFinite)).toBe(true);
    expect(
      Math.min(...body.getPositions().filter((_, i) => i % 3 === 1)),
    ).toBeLessThan(-0.05);
  }
});

it("sags cloth around fixed rest-space selections and writes its instance geometry", async () => {
  const world = await createWorld(earth);
  const source = new PlaneGeometry(2, 2, 4, 4).rotateX(-Math.PI / 2);
  const cloth = new SoftBody({
    shape: { kind: "surface", geometry: source, thickness: 0.01 },
    material: fabric,
  });
  const parent = new Group();
  parent.position.set(3, 2, 1);
  parent.add(cloth);
  const pin = new SoftAttachment({
    body: cloth,
    region: { min: [-2, -0.1, -1.1], max: [2, 0.1, -0.9] },
    target: null,
  });
  world.scene.add(parent, pin);
  world.update(0);
  const before = cloth.getPositions();
  steps(world, 30);
  const after = cloth.getPositions();
  expect(after.every(Number.isFinite)).toBe(true);
  let sagged = false;
  for (let i = 0; i < before.length; i += 3) {
    const initial = new Vector3().fromArray(before, i);
    const current = new Vector3().fromArray(after, i);
    if (initial.z < 0.1) expect(current.distanceTo(initial)).toBeLessThan(1e-5);
    else if (current.y < initial.y - 0.2) sagged = true;
    const rendered = new Vector3()
      .fromBufferAttribute(cloth.geometry.getAttribute("position"), i / 3)
      .applyMatrix4(cloth.matrixWorld);
    expect(rendered.distanceTo(current)).toBeLessThan(1e-5);
  }
  expect(sagged).toBe(true);
  expect(source.getAttribute("position").getY(12)).toBeCloseTo(0);
  world.reset();
  expect(cloth.getPositions()).toEqual(before);
});

it("transfers a rope impulse into its rigid mount and releases the attachment", async () => {
  const world = await createWorld();
  const bar = inertialBody();
  const rope = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [0, 0, 0],
        [0.25, 0, 0],
        [0.5, 0, 0],
        [0.75, 0, 0],
        [1, 0, 0],
      ],
      radius: 0.1,
    },
    material: { ...fabric, density: 1 / (Math.PI * 0.1 ** 2) },
  });
  const mount = new SoftAttachment({ body: rope, region: [0], target: bar });
  world.scene.add(bar, rope, mount);
  rope.applyImpulse(new Vector3(5, 0, 0), new Vector3(1, 0, 0));
  steps(world, 100);
  expect(bar.getVelocity().linear.x).toBeCloseTo(2.5, 2);
  expect(
    new Vector3().fromArray(rope.getPositions()).distanceTo(bar.position),
  ).toBeLessThan(1e-5);
  mount.setEnabled(false);
  world.update(0);
  rope.applyImpulse(new Vector3(2, 0, 0));
  steps(world, 40);
  expect(bar.getVelocity().linear.x).toBeCloseTo(2.5, 2);
  expect(rope.getPositions()[0]).toBeGreaterThan(bar.position.x + 0.1);
});

it("skins an automatically meshed solid, preserves deformation when its density changes, and resets", async () => {
  const world = await createWorld();
  const block = new SoftBody({
    shape: {
      kind: "volume",
      geometry: new BoxGeometry(1, 1, 1),
      cellSize: 0.5,
    },
    material: rubber,
  });
  block.position.set(1, 2, 3);
  const mount = new SoftAttachment({
    body: block,
    region: { min: [-0.8, -0.8, -0.8], max: [-0.4, 0.8, 0.8] },
    target: null,
  });
  world.scene.add(block, mount);
  world.update(0);
  const rest = block.getPositions();
  block.applyImpulse(new Vector3(0, 2, 0), new Vector3(1.5, 2.5, 3.5));
  steps(world, 10);
  const deformed = block.getPositions();
  expect(deformed).not.toEqual(rest);
  expect(
    Array.from(block.geometry.getAttribute("position").array).every(
      Number.isFinite,
    ),
  ).toBe(true);
  block.setMaterial({ ...rubber, density: 200 });
  world.update(0);
  expect(block.getPositions()).toEqual(deformed);
  world.reset();
  expect(block.getPositions()).toEqual(rest);
});

it("supports authored tetrahedra and follows a kinematic rigid mount", async () => {
  const world = await createWorld();
  const target = inertialBody({ bodyType: "kinematic" });
  const solid = new SoftBody({
    shape: {
      kind: "volume",
      positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
      tetrahedra: [0, 1, 2, 3],
    },
    material: rubber,
  });
  world.scene.add(
    target,
    solid,
    new SoftAttachment({ body: solid, region: [0], target }),
  );
  target.setKinematicTarget(new Matrix4().makeTranslation(0, 0.25, 0));
  steps(world, 1);
  const loaded = solid.getPositions();
  solid.setMaterial({ ...rubber, density: 200 });
  world.update(0);
  expect(solid.getPositions()).toEqual(loaded);
  steps(world, 3);
  expect(solid.getPositions()[1]).toBeCloseTo(0.25, 3);
  expect(solid.geometry.getAttribute("position").getY(0)).toBeCloseTo(0.25, 3);
});

it("reenables attachments at their captured world and rigid-local anchors", async () => {
  const world = await createWorld();
  const target = inertialBody({ bodyType: "static" });
  target.position.z = 2;
  world.scene.add(target);
  const setups = [null, target].map((target, index) => {
    const body = new SoftBody({
      shape: {
        kind: "curve",
        points: [
          [0, 0, 0],
          [1, 0, 0],
        ],
        radius: 0.1,
      },
      material: fabric,
    });
    body.position.z = index * 2;
    const mount = new SoftAttachment({ body, region: [0], target });
    world.scene.add(body, mount);
    return { body, mount };
  });
  world.update(0);
  for (const { mount } of setups) mount.setEnabled(false);
  world.update(0);
  for (const { body } of setups) body.applyImpulse(new Vector3(1, 0, 0));
  steps(world, 50);
  for (const { body, mount } of setups) {
    expect(body.getPositions()[0]).toBeGreaterThan(0.01);
    mount.setEnabled(true);
  }
  steps(world, 50);
  for (const { body } of setups)
    expect(body.getPositions()[0]).toBeCloseTo(0, 4);
});

it("rejects overlapping attachments and unsupported material models visibly", async () => {
  const world = await createWorld();
  const rope = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [0, 0, 0],
        [1, 0, 0],
      ],
      radius: 0.1,
    },
    material: fabric,
  });
  const first = new SoftAttachment({ body: rope, region: [0], target: null });
  const second = new SoftAttachment({ body: rope, region: [0], target: null });
  world.scene.add(rope, first, second);
  expect(() => world.update(0)).toThrow("cannot share simulation nodes");
  world.scene.remove(second);
  world.update(0);
  const solid = new SoftBody({
    shape: { kind: "volume", geometry: new BoxGeometry(), cellSize: 0.5 },
    material: {
      ...rubber,
      elasticity: {
        model: "saintVenantKirchhoff",
        youngModulus: 2000,
        poissonRatio: 0.3,
      },
    },
  });
  world.scene.add(solid);
  expect(() => world.update(0)).toThrow("does not support Saint Venant");
  solid.setMaterial({
    ...rubber,
    elasticity: {
      model: "neoHookean",
      youngModulus: 2000,
      poissonRatio: 0.3,
      dampingTime: 0.002,
    },
  });
  expect(() => world.update(0)).toThrow("does not support dampingTime");
});

it("collides deformables with rigid geometry and exposes them to queries", async () => {
  const world = await createWorld(earth);
  const floor = box("static");
  floor.position.y = -0.5;
  floor.scale.set(5, 1, 5);
  const solid = new SoftBody({
    shape: {
      kind: "volume",
      geometry: new BoxGeometry(0.5, 0.5, 0.5),
      cellSize: 0.25,
    },
    material: rubber,
  });
  solid.position.y = 1;
  world.scene.add(floor, solid);
  steps(world, 120);
  const positions = solid.geometry.getAttribute("position");
  let lowest = Infinity;
  for (let i = 0; i < positions.count; i++)
    lowest = Math.min(lowest, positions.getY(i) + solid.position.y);
  expect(lowest).toBeGreaterThan(-0.05);
  expect(lowest).toBeLessThan(0.1);
  expect(
    world.raycast(new Vector3(0, 3, 0), new Vector3(0, -1, 0), 5),
  ).toMatchObject({ body: solid });
  expect(
    solid.getContacts().some((contact) => contact.otherBody === floor),
  ).toBe(true);
  expect(
    floor.getContacts().some((contact) => contact.otherBody === solid),
  ).toBe(true);
});

it("keeps native deformation intact when a material update is unsupported", async () => {
  const world = await createWorld();
  const rope = new SoftBody({
    shape: {
      kind: "curve",
      points: [
        [0, 0, 0],
        [1, 0, 0],
      ],
      radius: 0.1,
    },
    material: fabric,
  });
  world.scene.add(rope);
  rope.applyImpulse(new Vector3(0, 1, 0), new Vector3(1, 0, 0));
  steps(world, 2);
  const before = rope.getPositions();
  rope.setMaterial(rubber);
  expect(() => world.update(0)).toThrow("require spring elasticity");
  expect(rope.getPositions()).toEqual(before);
  rope.setMaterial(fabric);
  steps(world, 1);
  expect(rope.getPositions()).not.toEqual(before);
});

it("keeps pending forces across a density rebuild and clears them after one step", async () => {
  const world = await createWorld();
  const make = () =>
    new SoftBody({
      shape: {
        kind: "curve",
        points: [
          [0, 0, 0],
          [1, 0, 0],
        ],
        radius: 0.1,
      },
      material: fabric,
    });
  const edited = make(),
    reference = make();
  reference.position.z = 2;
  const changed = { ...fabric, density: 1000 };
  reference.setMaterial(changed);
  world.scene.add(edited, reference);
  edited.applyForce(new Vector3(0, 10, 0));
  reference.applyForce(new Vector3(0, 10, 0));
  edited.setMaterial(changed);
  steps(world, 1);
  const first = edited.getPositions()[1];
  expect(first).toBe(reference.getPositions()[1]);
  steps(world, 3);
  expect(edited.getPositions()[1]).toBe(reference.getPositions()[1]);
});

it("interprets unlocated loads as total SI force and impulse independent of node count", async () => {
  const world = await createWorld();
  const ropes = [2, 5].map((count, index) => {
    const body = new SoftBody({
      shape: {
        kind: "curve",
        points: Array.from({ length: count }, (_, i) => [
          i / (count - 1),
          0,
          0,
        ]),
        radius: 0.1,
      },
      material: { ...fabric, density: 4 / (Math.PI * 0.1 ** 2) },
    });
    body.position.z = index * 2;
    world.scene.add(body);
    return body;
  });
  for (const body of ropes) body.applyImpulse(new Vector3(1, 0, 0));
  steps(world, 100);
  for (const body of ropes) expect(body.getPositions()[0]).toBeCloseTo(0.25, 4);
  world.reset();
  for (let step = 0; step < 100; step++) {
    for (const body of ropes) body.applyForce(new Vector3(1, 0, 0));
    steps(world, 1);
  }
  for (const body of ropes)
    expect(body.getPositions()[0]).toBeCloseTo(0.125, 2);
});
