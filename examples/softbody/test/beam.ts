export const length = 0.9;
export const width = 0.18;
export const thickness = 0.075;
export const youngModulus = 75e6;
export const density = 1200;

// Small-deflection Euler–Bernoulli reference for the rectangular specimen only.
export function theory(force: number, gravity: number) {
  const inertia = (width * thickness ** 3) / 12;
  return (
    (force * length ** 3) / (8 * youngModulus * inertia) +
    (density * width * thickness * gravity * length ** 4) /
      (8 * youngModulus * inertia)
  );
}
