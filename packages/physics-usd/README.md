# @drawcall/physics-usd

USD Physics import/export for Three.js authoring objects from `@drawcall/physics`.

```ts
import { PhysicsUSDExporter, PhysicsUSDLoader } from "@drawcall/physics-usd";

const bytes = await new PhysicsUSDExporter().parseAsync(scene);
const imported = await new PhysicsUSDLoader().parseAsync(bytes);
// imported is a Three.js Group with a gravity vector in m/s².
// When finished with the imported physics resources:
imported.dispose();
```

`PhysicsUSDExporter.parseAsync` returns `Uint8Array<ArrayBuffer>`, matching Three's `USDZExporter`. `PhysicsUSDLoader` accepts ASCII USDA strings, array buffers, and USDZ bytes. `parse` constructs synchronously; `parseAsync` also waits for referenced textures. `loadAsync(url)` fetches and imports a file.

## Representation

The exporter delegates visual geometry, materials, textures, cameras, and ZIP assets to Three.js `USDZExporter`. It adds a standard USDA root layer with a `subLayers` reference to the visual layer. Untyped definitions add physics schemas without replacing visual prim types. No patched Three.js source or serialized JavaScript physics metadata is used.

A temporary visual copy receives deterministic unique prim names. The original hierarchy is not mutated; standard USD `displayName` metadata retains original object names. The exported root includes physics materials, joints, and the physics scene so referencing the default prim includes the physical mechanism. Colliders come from each body’s `getColliders()` method and are resolved with the same scale conversion as Rapier. The temporary copy gives each body a rigid world transform, moves authored scale into its visual children, and exports baked invisible collision geometry beneath the body. Export resolves the fully composed hierarchy without requiring a simulation step. The exporter bakes the external ancestor transform into an attached export root, keeping world joint anchors consistent.

Supported mapping:

- Static, dynamic, and kinematic bodies; mass; initial linear/angular velocity.
- Compound boxes, spheres, capsules, cylinders, convex hull meshes, and static triangle meshes.
- Friction, restitution, and density through USD physics materials.
- Fixed, revolute, prismatic, unrestricted spherical, and bounded distance joints.
- Both body-local joint frames, world anchoring through `body0`, enable state, connected-body collision state.
- Revolute/prismatic position and velocity drives, force/acceleration mode, stiffness, damping, and force/torque limits.
- SI stage units and Y-up. Angular targets, limits, velocity, and drive coefficients convert between radians and USD's degree-based angular units.

Import creates actual `RigidBody`, collider, material, and joint instances. It resolves inherited physics material bindings and density and retains shared material identity. The returned `PhysicsUSDScene` extends Three.js `Group`; its `gravity` preserves the stage's gravity. Use `clone(scene)` from `@drawcall/physics` to remap joint references to cloned bodies while retaining the same world. Native `scene.clone()` follows Three.js behavior and retains original joint references. Disposing a clone releases its own registrations. By default each imported scene owns an `AuthoringWorld`: loading works without a default world and never replaces the application's default world. `scene.dispose()` releases the imported bodies and joints. Visual geometry and materials retain normal Three.js ownership; release any owned visual resources before `scene.dispose()`, which detaches bodies. Pass `new PhysicsUSDLoader({ world })` to register imported objects in a running simulation; disposing the imported scene releases only its objects and keeps that supplied world alive. A loading manager can be supplied as `{ manager }`.

## Explicit boundaries

This first importer supports **ASCII USDA and USDZ archives containing ASCII layers**, with multiline prim metadata and properties. Three's current USDC parser loses applied API schemas, so binary USDC is rejected. Convert binary assets to USDA using OpenUSD first.

ASCII import supports embedded sublayers and visual-only geometry references. Flatten physics-bearing references, `over`/`class` specs, variants, and other composition features before import. External reference layers must be embedded. Reset transform stacks and unsupported transform operations are rejected. `metersPerUnit=1`, `kilogramsPerUnit=1`, Y-up, and core-compatible rigid transforms are required. This is a bounded USD Physics reader, not a general OpenUSD composition engine.

The package rejects unsupported semantics rather than silently dropping them: sensors, collision masks, damping/gravity/sleep overrides, D6/articulations, spherical cone limits, breaking thresholds, manual center of mass/inertia, per-collider explicit mass, animated physics, and simulation ownership. Distance import requires a finite maximum; world anchoring must use body0. Animation export and `onlyVisible: true` are rejected: collision geometry must remain in the physical asset. Invisible objects retain their visibility opinions.

Export rejects assemblies spanning multiple worlds because simulation ownership
is not represented by this adapter. Export captures the supplied transforms. Reset a running simulation to its authored pose before exporting an authored asset.

## Verification

`pnpm test` performs real USDZ roundtrips and imports independently authored USDA. `pnpm typecheck` also checks tests. For independent OpenUSD validation, install Python `usd-core` and run:

```sh
node scripts/fixture.ts /tmp/door.usdz
python scripts/validate.py /tmp/door.usdz
```

## Source layout

`src/import/` owns USDA parsing, schema validation, and reconstruction of bodies,
colliders, and joints. `src/export/` owns USD prim writing and USDZ packaging.
`scene.ts` holds the imported scene and its resource ownership; `index.ts` is the
package's public API.

Static bodies export as transformed collider groups with `PhysicsMassAPI` and no
`PhysicsRigidBodyAPI`. Import reconstructs the group as `RigidBody({ type: "static" })`,
preserving compound colliders and joint targets. Dynamic and kinematic bodies
retain the rigid-body schema.
