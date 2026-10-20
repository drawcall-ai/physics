import { expect, test } from "vitest";
import { Box3, Scene, Vector3 } from "three";
import { SoftBody } from "@drawcall/physics";
import { buildWorld as mujocoWorld } from "@drawcall/physics-mujoco";
import { buildWorld as rapierWorld } from "@drawcall/physics-rapier";
import { cargo, hammock, simulationOptions } from "../model";

test.each([
  ["mujoco", "rubber"],
  ["mujoco", "foam"],
  ["rapier", "rubber"],
  ["rapier", "foam"],
] as const)(
  "%s catches rapid %s drops without inverted cells",
  async (backend, kind) => {
    const interval = 0.1;
    const model = hammock(backend);
    const scene = new Scene().add(model.root);
    const buildWorld = backend === "mujoco" ? mujocoWorld : rapierWorld;
    const world = await buildWorld({ scene, ...simulationOptions(backend) });
    const loads: SoftBody[] = [];
    const caught = new Set<SoftBody>();
    const stopSampling = world.onAfterStep(() => {
      for (const contact of model.cloth.getContacts())
        if (contact.otherBody instanceof SoftBody)
          caught.add(contact.otherBody);
    });
    let minimumVolumeRatio = Infinity;
    const advance = (seconds: number) => {
      for (let frame = 0; frame < Math.round(seconds * 60); frame++) {
        world.update(1 / 60);
        for (const body of loads) {
          const shape = body.options.shape;
          if (shape.kind !== "volume" || !("tetrahedra" in shape))
            throw new Error("Cargo requires a tetrahedral mesh");
          const positions = body.getPositions();
          for (let i = 0; i < shape.tetrahedra.length; i += 4) {
            const nodes = shape.tetrahedra.slice(i, i + 4);
            const ratio =
              signedVolume(positions, nodes) /
              signedVolume(shape.positions, nodes);
            minimumVolumeRatio = Math.min(minimumVolumeRatio, ratio);
          }
        }
      }
    };
    try {
      advance(3);
      for (let index = 0; index < 2; index++) {
        const body = cargo(backend, kind, index);
        if (!(body instanceof SoftBody))
          throw new Error("Cargo must be deformable");
        loads.push(body);
        scene.add(body);
        try {
          advance(interval);
        } catch (cause) {
          throw new Error(
            `Failed after ${kind} drop ${index + 1} at ${world.time}s`,
            { cause },
          );
        }
      }
      advance(5);
      expect(minimumVolumeRatio).toBeGreaterThan(0);
      for (const body of [model.cloth, ...loads])
        expect(body.getPositions().every(Number.isFinite)).toBe(true);
      for (const body of loads) {
        const bounds = new Box3().setFromArray(body.getPositions());
        const size = bounds.getSize(new Vector3());
        expect(Math.min(...size.toArray())).toBeGreaterThan(0.2);
        expect(Math.max(...size.toArray())).toBeLessThan(0.7);
        if (backend === "mujoco") expect(caught.has(body)).toBe(true);
        expect(bounds.min.y).toBeGreaterThan(-0.1);
      }
    } finally {
      stopSampling();
      world.dispose();
    }
  },
);

function signedVolume(positions: ArrayLike<number>, nodes: Iterable<number>) {
  const [i, j, k, l] = nodes;
  if (i === undefined || j === undefined || k === undefined || l === undefined)
    throw new Error("Incomplete tetrahedron");
  const a = new Vector3().fromArray(positions, i * 3);
  const b = new Vector3().fromArray(positions, j * 3).sub(a);
  const c = new Vector3().fromArray(positions, k * 3).sub(a);
  const d = new Vector3().fromArray(positions, l * 3).sub(a);
  return b.dot(c.cross(d)) / 6;
}
