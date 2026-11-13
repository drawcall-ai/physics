import { BoxGeometry, Mesh, MeshStandardMaterial } from "three";

export const thickness = 0.025;
const height = 0.08;
export const width = 0.06;

export const material = new MeshStandardMaterial({
  color: "#65cdbb",
  roughness: 0.75,
});

/** A pad mounted on its inner face at x = 0. */
export function pad(): Mesh {
  const geometry = new BoxGeometry(thickness, height, width, 1, 2, 2);
  geometry.translate(thickness / 2, 0, 0);
  return new Mesh(geometry, material);
}
