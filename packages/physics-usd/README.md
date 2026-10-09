# @drawcall/physics-usd

USD Physics import and export for the scene objects of
[`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme).
No simulation backend is needed.

```ts
import { PhysicsUSDExporter, PhysicsUSDLoader } from "@drawcall/physics-usd";

const bytes = await new PhysicsUSDExporter().parseAsync(scene, {
  gravity: [0, -9.81, 0], // default
});
const { scene: imported, gravity } = await new PhysicsUSDLoader().parseAsync(
  bytes,
);
```

## Export

`PhysicsUSDExporter.parseAsync(object, options)` returns USDZ bytes
(`Uint8Array<ArrayBuffer>`). Options are Three's `USDZExporter` options plus `gravity`.
Three's `USDZExporter` writes visuals, materials, and textures; physics is added as a
standard USDA root layer over them. The scene is not mutated, and object names survive
as USD `displayName`.

Export captures the current transforms. Call `world.reset()` first to export the
authored pose.

## Import

`PhysicsUSDLoader` accepts USDA strings, array buffers, and USDZ bytes with ASCII
layers. `parse` is synchronous, `parseAsync` also waits for textures, and
`loadAsync(url)` fetches a file. Pass `{ manager }` to the constructor for a
`LoadingManager`.

The result is `{ scene, gravity }`: `scene` is a `Group` of real `RigidBody`, collider,
and joint objects, and `gravity` is the stage gravity in m/s². It is valid world
options, so a stage can be simulated directly:

```ts
import { buildWorld } from "@drawcall/physics-rapier";

const world = await buildWorld({
  ...(await new PhysicsUSDLoader().loadAsync(url)),
  fixedDelta: 1 / 120,
});
```

Or add `scene` under an existing world's scene. Use `clone(scene)` from
`@drawcall/physics` to duplicate it with joints reconnected.

## Mapping

- Static, dynamic, and kinematic bodies; total or complete mass, center of mass, and
  inertia; linear and angular velocity. Static bodies export without
  `PhysicsRigidBodyAPI`.
- Box, sphere, capsule, cylinder, convex hull, and triangle mesh colliders
  (`convexDecomposition` approximation on moving bodies).
- Friction, restitution, and density as physics materials, including inherited
  bindings.
- Fixed, revolute, prismatic, spherical (no cone limit), distance, and generic joints,
  with both body-local frames, `body0` world anchors, enabled, and collide-connected
  state. A distance maximum of `Infinity` maps to USD's negative maximum. Generic joints
  are `PhysicsJoint` prims with a `PhysicsLimitAPI:<axis>` per axis: no limit is free,
  lower > upper is locked.
- Drives through `PhysicsDriveAPI`, per axis on generic joints. Distance joints use the
  linear drive, an extension other USD consumers ignore. A drive must have a target
  without effort; detach passive drives before export.
- SI units, Y-up. Angles convert to USD degrees.

## Rejected

- Binary USDC (Three's parser drops applied schemas); convert to USDA with OpenUSD.
- Physics-bearing references, `over`/`class` specs, variants, and other composition
  beyond embedded sublayers and visual-only references. Flatten first.
- Units other than meters and kilograms, non-Y-up stages, reset transform stacks,
  unsupported transform ops.
- Triggers, collision groups, damping, gravity scale, sleep settings, D6
  articulations, cone limits, breaking thresholds, partial or per-collider mass, and
  animated physics.
- Export with `animations` or `onlyVisible: true`.

## Verification

`pnpm test` runs USDZ roundtrips and imports independently authored USDA. With Python
`usd-core` installed, validate against OpenUSD:

```sh
node scripts/fixture.ts /tmp/door.usdz
python scripts/validate.py /tmp/door.usdz
```
