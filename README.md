# @drawcall/physics

Physics scene objects for Three.js, with Rapier simulation and USD interchange.
Bodies extend `Group`; colliders and joints extend `Object3D`.

| Package                    | Purpose                                                          |
| -------------------------- | ---------------------------------------------------------------- |
| `@drawcall/physics`        | Scene objects, validation, automatic colliders, assembly cloning |
| `@drawcall/physics-rapier` | Rapier simulation                                                |
| `@drawcall/physics-usd`    | USD Physics import and export                                    |

## Quick start

```sh
npm install @drawcall/physics @drawcall/physics-rapier three
```

```ts
import { BoxGeometry, Group, Mesh, MeshStandardMaterial } from "three";
import { RigidBody, RevoluteJoint } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";

const world = await setupWorld({ gravity: [0, -9.81, 0] });
const scene = new Group();
const material = new MeshStandardMaterial();
const frame = new RigidBody({ type: "static" });
frame.add(new Mesh(new BoxGeometry(0.1, 2.2, 0.15), material));

const door = new RigidBody({ mass: 20 });
door.position.x = 0.55;
door.add(new Mesh(new BoxGeometry(0.98, 2, 0.06), material));

const hinge = new RevoluteJoint({
  body0: frame,
  body1: door,
  limits: [0, Math.PI / 2],
});
hinge.position.x = 0.06;
scene.add(frame, door, hinge);

// Call from your render loop with elapsed time in seconds.
world.update(1 / 60);

// When the scene is no longer needed:
world.dispose();
```

## World ownership

Install a world before constructing physics objects. `setupWorld()` from a backend
package initializes the engine and installs its world. For authoring without
simulation, the core works without loading an engine:

```ts
import { AuthoringWorld, setDefaultWorld } from "@drawcall/physics";

setDefaultWorld(new AuthoringWorld());
```

Bodies capture the default world, or an explicit `{ world }` constructor option.
Joints inherit their connected bodies' world; cross-world connections fail.
Construction calls `world.register(object)`. A backend creates pending resources
before stepping, after geometry and initial transforms have been configured. New
bodies and joints can be constructed while the simulation is running.

Constructor settings live in `body.options` and `joint.options`. Their options
reference is readonly, and body references/world selection cannot be reassigned.
Mutable settings such as `hinge.options.drive` can change after construction.

`body.dispose()` releases its resources and connected joints. Removing a body from
its Three.js parent does not dispose it. `world.dispose()` releases all its objects
and clears the default only if that world is still the default. A later
`setupWorld()` affects new objects; existing bodies retain their original world.

## Objects

- `RigidBody extends Group`: `type` is `dynamic` (default), `static`, or
  `kinematic`. Optional total `mass` overrides shape density. Body materials,
  damping, velocities, gravity scale, and sleep policy are authored properties.
- `PhysicsMaterial`: plain options for static/dynamic friction, restitution, and density,
  e.g. `material: { density: 42 }`. Omitted values use the standard defaults.
- `BoxCollider`, `SphereCollider`, `CapsuleCollider`, `CylinderCollider`,
  `MeshCollider`: explicit `Object3D` shapes. Their presence disables automatic generation on their body.
  Their transforms locate shapes relative to the body. Capsules and cylinders
  extend along Y; capsule `length` excludes the hemispheres.
- `FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `SphericalJoint`,
  `DistanceJoint`: `Object3D` constraints. `body0: null` anchors to the world.
  Revolute/prismatic joints have an `axis` (default Y), optional `limits`, and
  optional `drive` with position/velocity targets, stiffness, damping, and force
  limit. Distance joints require minimum/maximum `limits`.

All quantities use meters, kilograms, seconds, and radians. Joint placement defines
both initial local frames. Supply both `frame0` and `frame1` instead for explicit
body-local `Matrix4` transforms. A world anchor uses a world-space `frame0`.

## Colliders, transforms, and cloning

`body.getColliders()` returns actual explicit or generated collider objects.
Their `matrixWorld` values locate collision shapes in the scene; the body itself
carries its world transform in `matrixWorld`. `body.getMaterial(collider)` resolves
collider overrides, body defaults, and the default physics material.
`joint.getFrame(index, target)` writes a body-local anchor into a supplied `Matrix4`.
Validation rejects invalid transforms, limits, and settings before adapters consume them.
`getColliders()` and `getFrame()` validate automatically; adapters can also call `validate()`
when checking mutable settings without generating shapes or recalculating anchors.
`resolveCollider(body, collider)` returns a rigid body-local matrix and a shape
with authored world scale baked into its dimensions or copied mesh vertices.
`collider.source` identifies the authored collider or mesh. Source geometry is not mutated.
`joint.getFrame()` accounts for scale in anchor positions; explicit frame rotations
remain body-local. Numeric limits, drive settings, mass, and velocities retain their
physical units.

Boxes support positive nonuniform scale. Spheres and capsules require uniform scale;
cylinders allow independent Y scale with equal X/Z scale. Mesh vertices support
positive nonuniform scale. Moving bodies require uniform ancestor scale, because
rotation beneath a nonuniform ancestor can introduce shear. Body-local nonuniform
scale is supported for compatible shapes.
Automatic colliders are generated per visual mesh: unchanged primitives retain their shapes;
other dynamic geometry uses a convex hull and other static geometry uses triangles.

Physics rejects zero/negative scale, shear, nested bodies, partial draw ranges,
and automatic colliders on instanced, skinned, or morph-deformed meshes. Triangle
colliders require static bodies. Explicit colliders take precedence over meshes; removing them restores automatic generation
unless `colliders: false` disables it. Standalone collision geometry requires a static body wrapper.

Bodies and joints support Three.js `.clone()` and `.copy()`, which preserve
joint body references. For a complete mechanism, use `clone(root)` from
`@drawcall/physics`, like Three.js SkeletonUtils: it clones an ordinary object
hierarchy and reconnects internal joint references to cloned bodies. External
body references remain external. Geometry and materials remain shared,
application-owned resources. Clones register in their source world.

Explicit colliders can declare `sensor: true` and
`collisionGroups: { membership, filter }` using unsigned 16-bit masks. Adapters
validate support and report unsupported properties instead of ignoring them.

## Adapters

Install `@drawcall/physics-rapier` for `await setupWorld()` and
`@drawcall/physics-usd` for `PhysicsUSDExporter` / `PhysicsUSDLoader`. Each has one
public entry point. The core exports `PhysicsWorld` so hosts can work with different backends.
Scene code calls physics methods directly on bodies and joints.

## Development

```sh
pnpm install
pnpm check
```

Run `pnpm --filter @drawcall/example-ragdoll dev` or
`pnpm --filter @drawcall/example-car dev`, then open the Vite URL.
The examples demonstrate an articulated ragdoll and a powered car with suspension
on a bump course. See [examples](https://github.com/drawcall-ai/physics/tree/main/examples)
for controls and source.

Use [tools/simulate.py](https://github.com/drawcall-ai/physics/tree/main/tools)
to open exported USDZ in Newton, record an MP4, or run a headless CPU check with
an independent importer and solver.

### State access and static previews

`body.getVelocity()`, `body.setVelocity({ linear, angular })`, `body.teleport(pose)`, and `joint.getState()` work during scene construction, including while a host stages objects outside world registration. Velocity defaults to zero. Input and output vectors are independent copies. Read transforms through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning; observation after a step needs no refresh. After direct authoring or hierarchy changes, call `body.updateWorldMatrix(true, false)` if reading immediately. That matrix includes scale; `splitTransform(body.matrixWorld).pose` gives a rigid pose for teleportation.

`AuthoringWorld` permits a static preview to run one scene callback without a simulation backend. Velocity is stored authored state; teleportation updates the object immediately. Valid forces, impulses, kinematic targets, and sleep/wake calls are explicitly inert; they do not move the object or alter velocity. Step observers can register/unsubscribe but never run. Invalid arguments and disposed/foreign objects still fail. Calling `update`, `step`, or `reset` on an authoring world still throws because it cannot simulate.

The Rapier adapter prepares completed assemblies at `update(0)` as well as timed updates and before-step boundaries. State operations never force early backend creation; final parent scale, collider geometry, mass, and inertia are captured together. See the [Rapier lifecycle contract](packages/physics-rapier/README.md) for simulation operations and reset semantics.
