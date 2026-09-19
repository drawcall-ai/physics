import { RigidBody } from "@drawcall/physics";
import {
  BoxGeometry,
  ExtrudeGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  Path,
  Shape,
} from "three";

/** Identical closed meshes; only their collision approximation differs. */
export function decomposition() {
  const outline = new Shape();
  outline.moveTo(-1.5, -1.5);
  outline.lineTo(1.5, -1.5);
  outline.lineTo(1.5, 1.5);
  outline.lineTo(-1.5, 1.5);
  outline.closePath();
  const hole = new Path();
  hole.moveTo(-0.85, -0.85);
  hole.lineTo(-0.85, 0.85);
  hole.lineTo(0.85, 0.85);
  hole.lineTo(0.85, -0.85);
  hole.closePath();
  outline.holes.push(hole);
  const geometry = new ExtrudeGeometry(outline, {
    depth: 0.4,
    bevelEnabled: false,
    steps: 1,
  });
  geometry.rotateX(-Math.PI / 2);
  const root = new Group();
  const floor = new RigidBody({ type: "static" });
  floor.position.y = -0.1;
  floor.add(
    new Mesh(
      new BoxGeometry(10, 0.2, 6),
      new MeshStandardMaterial({ color: "#526478" }),
    ),
  );
  root.add(floor);
  const lanes = (["convexHull", "trimesh"] as const).map(
    (approximation, index) => {
      const frame = new RigidBody({ type: "static", colliders: approximation });
      frame.name = approximation;
      frame.position.set(index === 0 ? -2 : 2, 1.8, 0);
      frame.add(
        new Mesh(
          geometry,
          new MeshStandardMaterial({
            color: index === 0 ? "#eaa65a" : "#83d9cb",
          }),
        ),
      );
      const cube = new RigidBody({ mass: 1 });
      cube.name = `${approximation} cube`;
      cube.position.set(frame.position.x, 4.5, 0);
      cube.add(
        new Mesh(
          new BoxGeometry(0.5, 0.5, 0.5),
          new MeshStandardMaterial({ color: "#eeeeff" }),
        ),
      );
      root.add(frame, cube);
      return { frame, cube };
    },
  );
  return { root, lanes };
}
