# @drawcall/physics

Physics scene objects for Three.js, with Rapier and MuJoCo WASM simulation and USD interchange.
Bodies extend `Group`; colliders and joints extend `Object3D`.

| Package                    | Purpose                                                          |
| -------------------------- | ---------------------------------------------------------------- |
| `@drawcall/physics`        | Scene objects, validation, automatic colliders, assembly cloning |
| `@drawcall/physics-rapier` | Rapier simulation                                                |
| `@drawcall/physics-mujoco` | MuJoCo WASM simulation in browsers and Node.js                   |
| `@drawcall/physics-usd`    | USD Physics import and export                                    |

## Quick start

```sh
npm install @drawcall/physics @drawcall/physics-rapier three
```

```ts
import { BoxGeometry, Group, Mesh, MeshStandardMaterial } from "three";
import { RigidBody, RevoluteJoint } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-rapier";

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

const world = await buildWorld({ gravity: [0, -9.81, 0] });

// Call from your render loop with elapsed time in seconds.
world.update(1 / 60);

// When the scene is no longer needed:
world.dispose();
```

For MuJoCo, install `@drawcall/physics-mujoco` and import its `buildWorld` instead.
See the [MuJoCo adapter](packages/physics-mujoco/README.md) for browser WASM asset
loading and engine-specific constraints. All examples offer a backend dropdown;
changing it restarts the simulation.

## Building a world

Bodies, joints, and triggers register immediately in the core's single `registry`.
No engine is required to construct, clone, import, or export a scene. Finish the
initial scene, then call `await buildWorld(options)` from your chosen backend.
The returned world is ready for queries and simulation; building does not advance time.

Building before scene construction also works. Subsequent registrations and
disposals are forwarded to the attached backend, which prepares changes before
stepping. Initial building gives backends an opportunity to optimize the full
scene: MuJoCo decomposes initial triangle meshes into convex parts, while later
mesh additions use one convex hull. There is no background decomposition.

Only one world may be built or building at a time. Failed builds leave authored
objects registered so they can be corrected and the build retried.

Constructor options are copied and typed readonly: body type/mass, collider dimensions,
joint bodies/frames/limits/dofs, and drive gains. `options` carries the resolved
defaults. Recreate objects to change them.
Mutable settings use methods: `setVelocity`, `setLinearDamping`, `setAngularDamping`,
`setGravityScale`, `setMaterial`, `setEnabled`, `setCollideConnected`, `setDrive`,
and a drive's `setTarget`.
Private state and readonly configuration use TypeScript; numeric physics and
external-input constraints are checked at runtime.

`body.dispose()` unregisters it and disposes connected joints and attached triggers.
Removing a body from its Three.js parent does not dispose it. `world.dispose()`
releases its objects, native resources, and registry attachment. `registry.clear()`
disposes registered scene objects, including when no world has been built.

## Objects

- `RigidBody extends Group`: `type` is `dynamic` (default), `static`, or
  `kinematic`. Optional total `mass` overrides shape density. Use methods for materials,
  damping, velocities, and gravity scale; sleep capability is fixed.
- `PhysicsMaterial`: plain options for static/dynamic friction, restitution, and density,
  e.g. `body.setMaterial({ density: 42 })`. Omitted values use the standard defaults.
- `BoxCollider`, `SphereCollider`, `CapsuleCollider`, `CylinderCollider`,
  `MeshCollider`: explicit `Object3D` shapes. Their presence disables automatic generation on their body.
  Their transforms locate shapes relative to the body. Capsules and cylinders
  extend along Y; capsule `height` excludes the hemispheres.
- `FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `SphericalJoint`,
  `DistanceJoint`, `GenericJoint`: `Object3D` constraints. `body0: null` anchors to
  the world. Revolute/prismatic joints have an `axis` (default Y) and optional
  `limits`. Distance joints use minimum/maximum `limits`; an `Infinity` maximum
  leaves the distance free. A generic joint declares each of its six `dofs`
  (`transX` … `rotZ`, USD's tokens) as `"locked"` (the default), `"free"`, or a
  `[min, max]` range in frame 0.
- `JointDrive`: a plain class, not an `Object3D`. The force law on one joint
  coordinate with immutable gains, force ceiling, and model, and a mutable target.
  Revolute, prismatic, and distance joints hold one drive through `setDrive(drive)`;
  a generic joint holds one per axis through `setDrive(axis, drive)`.

Joint placement defines
both initial local frames. Supply both `frame0` and `frame1` instead for explicit
body-local `Matrix4` transforms. A world anchor uses a world-space `frame0`.

## Conventions

These hold for every backend. Adapters reject what they cannot honor instead of
approximating. The Rapier package's tests are the executable form of this contract:
a new backend starts from them and keeps its engine limitations in a test next to
the rejection, as `packages/physics-rapier/test/rapier.test.ts` does.

- Units are SI throughout the core: meters, kilograms, seconds, newtons, and
  radians. Adapters convert where a format defaults to degrees, as USD and MJCF do.
- Body damping is a rate in 1/s and independent of mass, as in PhysX, Jolt, Bullet,
  and Rapier. Engines with force-per-velocity damping scale it by mass and inertia.
- Linear velocity is measured at the center of mass. Angular velocity and the
  `point` of `applyForce`/`applyImpulse` are world space.
- `body0` is the base side of a joint and `null` is the world; `body1` is the moving
  side. Reduced-coordinate backends derive their kinematic tree from this graph and
  treat loop-closing joints as soft constraints, so prefer authoring a tree.
- Friction and restitution combine rules and contact softness are backend-defined.
  The core stores coefficients per body and collider only.
- The `acceleration` drive model is native to PhysX-like solvers and emulated with
  effective mass elsewhere. A backend that can do neither rejects it.
- A drive target replaces all three terms at once, so a controller that sets one
  term relies on the others reading as zero. Backends may cap the native
  stiffness/damping terms and the effort term separately rather than their sum.
- Immutable configuration uses tuples; runtime state uses Three.js vectors,
  quaternions, and matrices.
- Collider parameters use the physics-engine names: capsules and cylinders both
  take a `height`, boxes take a full `size`.

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
body references remain external. Copy requires matching immutable configuration;
cloning recreates drives through their own constructors, so subclasses survive. Geometry and materials remain shared,
application-owned resources. Clones register in their source world.

Explicit colliders use `setMaterial(...)` and
`setCollisionGroups({ membership, filter })` with unsigned 16-bit masks.
RigidBody and Trigger also provide `setCollisionGroups(...)` defaults. A collider
override replaces the complete owner default; passing `undefined` clears an override.
Both sides must allow a pair: `(a.membership & b.filter) !== 0 &&
(b.membership & a.filter) !== 0`. Trigger defaults do not inherit from an ancestor body.
Dimensions use constructor options, e.g. `new BoxCollider({ size: [1, 2, 3] })`.
Mesh geometry uses `setGeometry`; its approximation remains immutable. Adapters
validate support and report unsupported properties instead of ignoring them.

## Adapters

Worlds expose `disposed`, and `joint.connects(body)` tells whether a body is either
side of a joint. The core also exports adapter helpers (authored velocity and joint
readings, world-pose writeback, validation, and the shared world assertions) that
scene code never needs.

Install `@drawcall/physics-rapier` for `await buildWorld()` and
`@drawcall/physics-usd` for `PhysicsUSDExporter` / `PhysicsUSDLoader`. Each has one
public entry point. The core exports `PhysicsWorld` so hosts can work with different backends.
Scene code calls physics methods directly on bodies and joints.

## Development

```sh
pnpm install
pnpm --filter @drawcall/physics-mujoco build:coacd
pnpm check
```

The CoACD build downloads pinned native sources and installs Emscripten 5.0.2
if needed. It requires Bash, curl, tar, CMake, Node.js, and Python 3. Its inputs
are temporary and its output is gitignored; rerun it after cleaning generated
files or changing the build script. CI runs it before checks and publishing.

Run `pnpm --filter @drawcall/example-ragdoll dev` or
`pnpm --filter @drawcall/example-car dev`, then open the Vite URL.
The examples demonstrate an articulated ragdoll and a powered car with suspension
on a bump course. See [examples](https://github.com/drawcall-ai/physics/tree/main/examples)
for controls and source.

### State access and static previews

`body.getVelocity()`, `body.setVelocity({ linear, angular })`, `body.teleport(pose)`, and authored joint reads work during scene construction, before a world has been built. Velocity defaults to zero. Input and output vectors are independent copies. Read transforms through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning; observation after a step needs no refresh. After direct authoring or hierarchy changes, call `body.updateWorldMatrix(true, false)` if reading immediately. That matrix includes scale; `splitTransform(body.matrixWorld).pose` gives a rigid pose for teleportation.

Before building, velocity setters store authored state and teleportation updates
the object immediately. Forces, impulses, kinematic targets, sleep/wake, and
queries require a built world and throw otherwise. There is no authoring-world
stand-in and no `{ world }` option on scene objects.

The Rapier adapter prepares completed assemblies at `update(0)` as well as timed updates and before-step boundaries. Reads and writes need no preparation call. Temporary backend bodies evaluate pending impulses and queries without capturing final parent scale, collider geometry, mass, or joint anchors. See the [Rapier lifecycle contract](packages/physics-rapier/README.md) for simulation operations and reset semantics.

## Drives and readings

```ts
const drive = new JointDrive({ stiffness: 100, damping: 10, maxForce: 20 });
hinge.setDrive(drive);
drive.setTarget({ position: 0.5 }); // servo
drive.setTarget({ velocity: 2 }); // motor
drive.setTarget({ effort: 0.5 }); // torque input
drive.setTarget(undefined); // passive, no braking
hinge.setDrive(undefined); // detach; drive.joint becomes undefined
```

A drive applies `stiffness · (position − q) + damping · (velocity − q̇) + effort`,
capped by `maxForce`, to its joint coordinate. It exerts no force without a target.
Each `setTarget` replaces all three terms and zeroes the omitted ones; a nonzero
position needs stiffness and a nonzero velocity needs damping. A drive belongs to
one joint at a time. `model` defaults to `"force"`; `"acceleration"` is
backend-dependent. `maxForce` is N for translations or N·m for rotations; omission
means unbounded. Subclass `JointDrive` to carry application data with the drive:
joint copies and assembly clones reconstruct the subclass.

A drive with a constant target is a spring. A distance joint with an unlimited
maximum and a drive toward zero is a force-limited tether, and a free generic joint
with translation drives is how to drag a dynamic body without making it kinematic:
a kinematic hand follows the pointer or controller, and the capped drives pull the
body after it, so walls and floors still stop the body. A controller with
orientation adds drives on the rotation axes.

```ts
const hand = new RigidBody({ type: "kinematic", colliders: false });
const hold = new GenericJoint({
  body0: hand,
  body1: sword,
  frame0: new Matrix4(),
  frame1: new Matrix4().makeTranslation(sword.worldToLocal(grabPoint)),
  dofs: {
    transX: "free",
    transY: "free",
    transZ: "free",
    rotX: "free",
    rotY: "free",
    rotZ: "free",
  },
});
for (const axis of ["transX", "transY", "transZ"] as const)
  hold.setDrive(
    axis,
    new JointDrive({
      model: "acceleration",
      stiffness: 1000,
      damping: 63,
      maxForce: 600,
    }).setTarget({ position: 0 }),
  );
hand.setKinematicTarget(controllerPose); // each frame; hand.dispose() releases
```

The ragdoll example wires this to `@pmndrs/pointer-events` for mouse and touch.

Axis `getState()` returns `{ position, velocity }` in radians/rad·s⁻¹ or meters/m·s⁻¹.
Spherical `getState()` returns `{ rotation, angularVelocity }`: frame 1 relative to
frame 0 and the relative angular velocity in frame 0 coordinates. Distance joints
return `{ distance, velocity }`. Generic joints take the axis, `getState("rotY")`,
reading rotations as wrapped XYZ Euler angles of the relative rotation. Fixed
joints have no state. Backends deliver one frame-0 reading per joint
(`translation`, `rotation`, `linearVelocity`, `angularVelocity`, continuous `angle`)
from which every typed state derives.
Revolute position tracks turns each substep; motion must remain below π per substep.
Teleport rebases without counting turns; reset restores the initialized coordinate.
Rapier position-drive targets use continuous radians and must remain less than π
from the current position; longer trajectories need intermediate targets.
Body linear velocity is measured at COM, with both velocity vectors world-aligned.
Rapier reads rotating-slider velocity using native COM inference without stepping.
Reading before building requires explicit mass properties for this read and otherwise throws.
`world.time` counts completed substeps, advances before `onAfterStep`, and resets to
zero. Catch-up time discarded by `maxSubsteps` is not simulated time.

## Mass and raycasts

Mass options support collider-derived defaults, a total `mass` override, or complete
`mass`, `centerOfMass`, and `diagonalInertia` values. Optional `principalAxes` defaults
to identity. Tuples are body-local: COM in meters, inertia in kg·m², axes as quaternion
`[x,y,z,w]`. Other partial combinations fail. Explicit properties are authoritative
and are not scaled by visual transforms. Dynamic colliderless bodies need complete
positive mass/inertia; static and kinematic bodies can be colliderless.

`world.raycast(origin, direction, maxDistance, options)` returns the closest hit or
null. Hits contain `distance`, world `point`/`normal`, and source `collider`/mesh.
Narrow on `hit.kind`: `"body"` hits contain `body`, and `"trigger"` hits contain
`trigger`. Directions are normalized; distances are meters. Options include
`collisionGroups`, `excludeBodies`, and `includeTriggers` (default false).
`excludeBodies` excludes solid body hits, not attached Trigger hits. Query masks
use the same mutual rule as simulation. Inside-origin rays return the exit surface.
Queries include the current authored scene before the first update, without advancing time.
Raycasts require a built world.

## Triggers and contact events

`Trigger extends Group` owns an explicit compound detection region. Add ordinary
colliders beneath it, optionally through Groups. Its shapes detect overlap without
mass, inertia, forces, or collision response. Triggers have no automatic mesh
colliders and reject physical materials, triangle mesh regions, nested Triggers,
and rigid bodies beneath them. Supported convex shapes depend on the adapter.

```ts
import { BoxCollider, Trigger } from "@drawcall/physics";

const goal = new Trigger(); // registers with the scene registry
goal.position.set(0.5, 0.8, 0);
goal.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
scene.add(goal);

goal.addEventListener("enter", ({ body }) => {
  if (body === gripper) console.log("Gripper entered the target");
});
goal.addEventListener("exit", ({ body }) => {
  if (body === gripper) console.log("Gripper left the target");
});

gripper.addEventListener("contactbegin", ({ otherBody }) => {
  console.log("Contact started", otherBody);
});
gripper.addEventListener("contactend", ({ otherBody }) => {
  console.log("Contact ended", otherBody);
});
```

Events use typed Three.js event maps, preserving `added`/`removed` and typed
`target`/`type`. Trigger events belong only to the Trigger. They aggregate by
other rigid body: enter when its first shape pair overlaps, exit when the last
pair ends. Body contact events aggregate by body pair and reach both bodies.
Moving between compound shapes within a step does not create an extra exit/enter.
Contact lifecycle is simulation contact state, not a force measurement or a
promise to report every geometric intersection between immovable bodies.

A Trigger can be placed beneath a RigidBody and follows that body during the
same simulation step. It excludes its ancestor body; other bodies, including
joint neighbors, remain eligible under collision masks. Joint contact suppression
does not disable Trigger sensing. Triggers detect static, kinematic, dynamic,
and initially sleeping bodies; they do not detect other Triggers.

`goal.overlaps(gripper)` and `goal.getOverlappingBodies()` read the latest
completed-step observations. Returned arrays are snapshots. Reads do not run
fresh geometry tests or guarantee sampling at final integrated poses. Before the
first step the set is empty; `update(0)` does not populate it. Events dispatch
after transforms and overlap state synchronize, before `onAfterStep`. Late listeners
receive no replay. Triggers can be authored before building; overlap reads require a built world.

For a robot evaluation, require the gripper to occupy the region and remain
nearly stationary for half a simulated second, within ten simulated seconds:

```ts
let elapsed = 0;
let settledFor = 0;
let result: "pending" | "passed" | "failed" = "pending";
const stopEvaluation = world.onAfterStep((dt) => {
  if (result !== "pending") return;
  elapsed += dt;
  const velocity = gripper.getVelocity();
  const settled =
    goal.overlaps(gripper) &&
    velocity.linear.length() < 0.02 &&
    velocity.angular.length() < 0.1;
  settledFor = settled ? settledFor + dt : 0;
  if (elapsed <= 10 && settledFor >= 0.5) result = "passed";
  else if (elapsed >= 10) result = "failed";
});
// At episode teardown: stopEvaluation(); goal.dispose();
```

Reset the evaluator's own state for each episode. `gripper` is one RigidBody,
not the whole articulated robot. Overlap does not prove complete containment or
correct orientation; precision tasks also need world pose tolerances. Discrete
sampling can miss fast crossings.

Sleep and backend collider rebuilds do not create false exits. Disposal removes
active relationships and notifies surviving owners; scene detachment alone does
not dispose physics. Disposing a body also disposes attached Triggers. Reset clears
overlap state silently; the next step establishes fresh pairs. World disposal is
silent. Clone/copy preserve authored settings, not listeners or runtime overlaps.

Scene edits inside listeners remain synchronous; backend reconciliation waits
until the event batch ends. Disposed recipients are skipped. Listener exceptions
propagate and abort remaining delivery and `onAfterStep`, without replay or sample
rollback. Reentrant update/reset fails. Trigger export is rejected because core
USD Physics has no standard trigger-volume representation; authored collision
mask conversion is also unsupported by the USD adapter.
