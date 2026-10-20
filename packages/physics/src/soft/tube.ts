import { Curve, TubeGeometry, Vector3 } from "three";

class Polyline extends Curve<Vector3> {
  constructor(private readonly points: readonly Vector3[]) {
    super();
  }
  getPoint(t: number, target = new Vector3()): Vector3 {
    const index = Math.min(
      Math.floor(t * (this.points.length - 1)),
      this.points.length - 2,
    );
    const a = this.points[index],
      b = this.points[index + 1];
    if (!a || !b) throw new Error("Curve needs at least two points");
    return target.lerpVectors(a, b, t * (this.points.length - 1) - index);
  }
  override getPointAt(t: number, target = new Vector3()): Vector3 {
    return this.getPoint(t, target);
  }
}

export function tube(points: readonly Vector3[], radius: number): TubeGeometry {
  return new TubeGeometry(
    new Polyline(points),
    points.length - 1,
    radius,
    8,
    false,
  );
}
