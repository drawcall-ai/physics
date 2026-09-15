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
const frame = new RigidBody().setType("static");
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

Constructor options are copied and readonly for the object’s lifetime: world, collider strategy, mass properties, sleep capability, connected bodies, axis,
physical joint limits, and explicit frames. Mutable settings use methods, during construction and simulation.
For example, call `body.setLinearDamping(0.1)`, `body.setGravityScale(1)`,
`body.setType("kinematic")`, or `hinge.setEnabled(false)`. Read current values through
getters such as `body.linearDamping` and `hinge.limits`; returned values are independent.

`body.dispose()` releases its resources and connected joints. Removing a body from
its Three.js parent does not dispose it. `world.dispose()` releases all its objects
and clears the default only if that world is still the default. A later
`setupWorld()` affects new objects; existing bodies retain their original world.

## Objects

- `RigidBody extends Group`: `bodyType` is `dynamic` (default), `static`, or
  `kinematic`. Optional total `mass` overrides shape density. Set materials,
  damping, velocities, gravity scale, and type through methods. Sleep capability is fixed.
  A type transition clears velocity, forces, and pending kinematic targets; setting
  the same type preserves them. Set new velocity/targets after changing type.
- `PhysicsMaterial`: plain options for static/dynamic friction, restitution, and density,
  e.g. `body.setMaterial({ density: 42 })`. Omitted values use standard defaults.
- `BoxCollider`, `SphereCollider`, `CapsuleCollider`, `CylinderCollider`,
  `MeshCollider`: explicit `Object3D` shapes. Their presence disables automatic generation on their body.
  Their transforms locate shapes relative to the body. Capsules and cylinders
  extend along Y; capsule `length` excludes the hemispheres.
- `FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `SphericalJoint`,
  `DistanceJoint`: `Object3D` constraints. `body0: null` anchors to the world.
  Revolute/prismatic joints have immutable `axis` (default Y) and optional `limits`
  constructor options, plus `setEffort()`. Distance joints use immutable
  `limits: [minimum, maximum]`. Recreate a joint to change mechanical limits.
- `JointMotor`: a separate actuator bound to one revolute/prismatic joint. Native
  backend motors implement position/velocity control; robotics supplies commands
  and optional custom controllers.

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
remain body-local. Numeric limits, mass, and velocities retain their
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
body references remain external. Copying onto an existing physics object requires
matching immutable configuration; use `clone(root)` to rebuild a mechanism with new
body identities. Geometry and materials remain shared,
application-owned resources. Clones register in their source world.

Explicit colliders use `setSensor(true)`, `setMaterial(...)`, and
`setCollisionGroups({ membership, filter })` with unsigned 16-bit masks. Shape
dimensions are immutable constructor options, for example
`new BoxCollider({ size: [1, 2, 3] })` or
`new CapsuleCollider({ radius: 0.2, length: 1 })`. Recreate a collider to change
its dimensions. Meshes use `setGeometry` with an immutable constructor
`approximation`. Three.js transforms
and supported geometry/child edits remain available. Adapters
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

`body.getVelocity()`, `body.setVelocity({ linear, angular })`, `body.teleport(pose)`, and authored joint-state reads work during scene construction, including while a host stages objects outside world registration. Velocity defaults to zero. Linear velocity is measured at the center of mass; both velocity vectors use world-space axes. Input and output vectors are independent copies. Read transforms through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning; observation after a step needs no refresh. After direct authoring or hierarchy changes, call `body.updateWorldMatrix(true, false)` if reading immediately. That matrix includes scale; `splitTransform(body.matrixWorld).pose` gives a rigid pose for teleportation.

A rotating prismatic joint needs velocity at its anchors. When that depends on
collider-derived mass properties, call `world.update(0)` after completing assembly,
or let the next normal update prepare it. Earlier `getState()` calls throw a clear
preparation error; they never infer mass or initialize the backend. Nonrotating
slider reads and reads with explicit `centerOfMass` remain available during
authoring. Prepared measurements use the backend's point-velocity query.

`AuthoringWorld` permits a static preview to run one scene callback without a simulation backend. Velocity is stored authored state; teleportation updates the object immediately. Valid forces, impulses, kinematic targets, and sleep/wake calls are explicitly inert; they do not move the object or alter velocity. Step observers can register/unsubscribe but never run. Invalid arguments and disposed/foreign objects still fail. Calling `update` or `reset` on an authoring world still throws because it cannot simulate.

The Rapier adapter prepares completed assemblies at `update(0)` as well as timed updates and before-step boundaries. State operations never force early backend creation; final parent scale, collider geometry, mass, and inertia are captured together. See the [Rapier lifecycle contract](packages/physics-rapier/README.md) for simulation operations and reset semantics.

## Native joint motors

```ts
const motor = new JointMotor({
  joint: hinge,
  stiffness: 100,
  damping: 10,
  maxForce: 20,
});
motor.setTarget({ position: 0.5, velocity: 0 });
motor.setEnabled(false);
motor.dispose(); // joint remains
```

Import `JointMotor` from `@drawcall/physics`. Binding, gains, maximum effort, and
`model` (`"force"` by default, or `"acceleration"` where supported) are immutable.
One motor may bind to each supported axis joint. Targets persist until replaced;
`setTarget` accepts position, velocity, or both; omitted coordinates become zero.
Configured gains remain active, so a pure velocity motor uses zero stiffness. Motors start enabled without a
target and produce no actuation until a target is supplied. Zero velocity with
damping brakes; disabling removes actuation. Joint/world disposal releases motors.
Construction never creates a backend early. Native motors run each solver substep.
Force-mode maximum effort is in N for sliders and N·m for hinges; omitted `maxForce`
is unbounded. Unsupported backend models fail explicitly. Equal settings do not
promise identical trajectories across engines.

## Joint measurements, effort, and time

Fixed-joint state contains relative `translation` and `rotation`; spherical and
distance joints report anchor `distance`.

Revolute and prismatic `getState()` return `{ position, velocity }`, in radians and
radians/second or meters and meters/second respectively. Revolute position counts
turns each solver substep, even without reads. Motion must stay below half a turn
per substep for unambiguous angular sampling. Teleports rebase the coordinate
without counting a jump as traveled turns; reset restores the initialized coordinate.

`joint.setEffort(value)` commands torque (N·m) or force (N) for one fixed substep.
The final call wins; zero cancels. No new command means zero effort next substep.
Positive effort increases the coordinate and applies the reaction to the connected
body. Disabled joints do not apply effort; disable, reset, and disposal clear it.
Effort requires a prepared joint: finish assembly and call `update(0)`, or submit
commands from the first `onBeforeStep` callback. Unprepared commands fail instead
of being queued. No-step updates do not consume a prepared command.
Disable an active motor before submitting nonzero effort; combined motor and
feed-forward effort is unsupported. Zero still cancels a pending command.
AuthoringWorld validates effort without retaining commands.

```ts
world.onBeforeStep(() => {
  const { position, velocity } = hinge.getState();
  hinge.setEffort(10 * (target - position) - 2 * velocity);
});
world.onAfterStep(() => sampleSensors(world.time));
```

`world.time` starts at zero and advances only after completed fixed substeps.
Before-step callbacks see starting time; after-step callbacks see completed time
and synchronized state. Sub-timestep updates and discarded catch-up time do not
advance it. Reset restores zero; AuthoringWorld always reports zero.

## Explicit mass and range queries

Body options `centerOfMass` (meters), `diagonalInertia` (kg·m²), and `principalAxes`
(unit quaternion `[x, y, z, w]`) describe body-local mass properties alongside
`mass` (kg). Principal moments must be positive for dynamic bodies and satisfy the
inertia triangle inequalities. A complete specification is authoritative; colliders
do not contribute mass twice. Supply either no mass properties, a total `mass`
override, or complete `mass`, `centerOfMass`, and `diagonalInertia` values
(`principalAxes` defaults to identity). Other partial combinations fail.
The backend derives mass properties from colliders when no complete override exists.
A dynamic body without colliders requires explicit positive mass and inertia.
Static and kinematic bodies can have no colliders without invented mass. Explicit
COM and inertia already use physical units and are not multiplied by visual scale.
Collider-derived values include captured collider scale. With a mass-only override,
collider mass contributions keep their density ratios (or volume ratios when all
densities are zero).

```ts
const link = new RigidBody({
  colliders: false,
  mass: 2,
  centerOfMass: [0, 0.1, 0],
  diagonalInertia: [0.02, 0.03, 0.04],
  principalAxes: [0, 0, 0, 1],
});
world.update(0);
const hit = world.raycast(origin, direction, 10, {
  excludeBodies: [link],
  includeSensors: false,
});
```

Raycasts return the closest hit or `null`, including distance, world point/normal,
body, and source collider or mesh identity. Direction is normalized internally;
distance is in meters. Collision groups can filter queries; sensors are excluded
by default. Rays starting inside a collider return its exit surface. Queries use
prepared state, including completed steps and teleports; use `update(0)` to prepare
completed construction. Queries never initialize unfinished bodies. AuthoringWorld
throws an unsupported-query error.

## Breaking migration

Release core, Rapier, and USD together under the next minor version using the
existing shared release tag workflow. Migrate consumers before upgrading:

| Previous API                         | Replacement                                                                  |
| ------------------------------------ | ---------------------------------------------------------------------------- |
| Velocity options                     | `body.setVelocity({ linear, angular })` with Vector3 values                  |
| Damping, gravity, material options   | `setLinearDamping`, `setAngularDamping`, `setGravityScale`, `setMaterial`    |
| Writable collider properties/options | Constructor options for dimensions; methods for material, sensor, and groups |
| Joint enabled/contact options        | `setEnabled`, `setCollideConnected`; limits remain constructor options       |
| Joint drive options / JointDrive     | `new JointMotor({ joint, ...configuration })` and `motor.setTarget(...)`     |
| Axis angle/angularVelocity fields    | `getState().position` / `.velocity`                                          |
| Body type option                     | `body.setType(type)`; mass and joint frames remain immutable                 |
| USD PhysicsDriveAPI                  | Preserved through `JointMotor`                                               |

There are no aliases for removed APIs. USD interchange carries physical constraints
motors and mass properties; robotics owns command timing, controller restrictions, custom actuator models, and ROS integration.

See [physics API decisions](https://github.com/drawcall-ai/physics/blob/main/docs/physics-api.md) for the engine conventions behind
mass inference, preparation, and one-step effort commands.
