# @drawcall/physics-rapier

Rapier implementation of `PhysicsWorld`.

```ts
import { buildWorld } from "@drawcall/physics-rapier";
import { RigidBody, BoxCollider } from "@drawcall/physics";
import { BoxGeometry, Mesh, Vector3 } from "three";

const body = new RigidBody({ mass: 1 });
body.add(new Mesh(new BoxGeometry(1, 1, 1)));
scene.add(body);
const world = await buildWorld({ gravity: [0, -9.81, 0] });
world.update(deltaSeconds);
```

`buildWorld()` initializes Rapier, attaches to the single physics registry, and prepares all registered objects without advancing time. Create the initial scene first; building an empty world and adding objects later also works. Only one world can be attached at a time.

`buildWorld({ solverIterations: 16 })` increases constraint solver precision for demanding joint chains, such as vehicle wheel assemblies. The value must be a positive integer; omitting it preserves Rapier’s default. Higher values cost more CPU time.

Dynamic bodies need colliders or complete explicit mass properties; static and kinematic bodies may be colliderless. Construction and registration never create a backend body. Complete geometry, scale, and parenting before the next `world.update(delta)`. Even `update(0)` and sub-timestep updates prepare bodies, colliders, mass properties, and joints without advancing simulation time. Pending objects are also prepared before before-step callbacks; changes and objects created in those callbacks are synchronized before the solver runs.

New objects do not reset existing simulation state. Colliders follow child additions/removals, geometry changes, collider properties and materials. Body damping and gravity scale update through methods; body type and `canSleep` are immutable.

Body and collider scale are captured once; later scale edits throw and require disposing and recreating the affected bodies and joints. New colliders capture their scale when added. Explicit body mass stays fixed; density-derived mass and inertia follow the scaled shapes.

Joint anchors are captured on first materialization. Explicit `frame0` and `frame1` options are Three.js `Matrix4` transforms relative to their respective bodies (or world space for `body0: null`). Drives, their targets, and connected-contact settings update before each step; limits are immutable. Changing joint transforms afterward does not move captured anchors. Explicit frame options are copied at construction, and the getter returns defensive matrix copies. Editing those copies does not change the anchors; dispose and create a new joint to change them. Recreate a joint to change its limits. Disable/re-enable with `joint.setEnabled(value)`.

Use methods directly on the objects:

```ts
const body = new RigidBody({ mass: 2 });
body.setVelocity({ linear: new Vector3(2, 0, 0) });
body.getVelocity(); // available immediately, before colliders or backend
body.add(new BoxCollider());
scene.add(body);
body.applyImpulse(new Vector3(1, 0, 0));
world.update(world.fixedDelta); // first ordinary step applies pending commands
```

`getVelocity()` returns independent linear/angular vectors. Missing initial components default to zero; partial `setVelocity()` calls preserve the other component and copy their inputs. Before initialization, setters update authored initial velocity; afterward they update the backend's current velocity. `reset()` restores the pose and velocity captured at initialization, including construction-time setters. Later runtime operations do not rewrite that baseline.

Read the object's pose through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning, so observation callbacks after a step need no refresh. Call `body.updateWorldMatrix(true, false)` only when reading immediately after direct authoring or hierarchy changes. Copy or clone it to retain a snapshot. It includes scale; `splitTransform(body.matrixWorld).pose` extracts the rigid pose. The old `world.body`, `world.joint`, and `getMatrix` APIs are removed. `teleport(matrix)` works before and after initialization and synchronously updates the object's world pose while preserving world scale. Under a nonuniformly scaled static parent, the local scale may change to compensate; transforms requiring shear are rejected before the object or backend is moved. Both it and `setKinematicTarget(matrix)` require a world-space rigid matrix with unit scale and no shear. After initialization the backend owns the dynamic body's pose; assigning Three.js position alone does not teleport it.

Commands work before the first update. Pending body commands replay in call order against the completed assembly, before `onBeforeStep` callbacks. Impulses add velocity once; a later velocity setter replaces the specified component. Forces add for one solver substep and survive no-step updates. Sleep/wake and kinematic targets retain their ordering. Reset and disposal clear pending commands; reset also clears a kinematic body's next target. `onBeforeStep` and `onAfterStep` return unsubscribe functions.

Reads and raycasts never simulate or capture permanent geometry, scale, or joint anchors. Before preparation, Rapier uses disposable bodies to evaluate inferred mass, pending impulses, and ray intersections from the current assembly. A force does not change velocity until a solver step. Later construction edits remain visible. Missing physical data, invalid frames, and disposed/foreign objects fail clearly.

Axis `joint.getState()` returns position/velocity immediately, including rotating sliders with inferred COM. Disabled joints remain readable using captured frames once prepared.

This is a breaking API change: release the packages together under a new minor version and migrate scene consumers before deploying that release.

`body.dispose()` unregisters its physics resources and connected joints. Removing a visual from its parent does not dispose physics. `world.dispose()` disposes all registered physics objects and frees Rapier. Geometry and materials remain owned by the application.

Rapier's own limits are tested in `test/rapier.test.ts`: equal static/dynamic friction, distance joints with a zero minimum, revolute position targets within π of the current angle, and positive integer solver iterations. Distance joints are Rapier spring joints: a finite maximum becomes their rope limit, and a `JointDrive` acts on the spring's coupled linear axis. Generic joints map to Rapier generic joints with per-axis limits and motors. Unsupported authored data fails visibly.

`update(delta)` accumulates elapsed seconds and runs fixed simulation steps, up to the configured `maxSubsteps` catch-up limit. `update(0)` prepares without advancing time. For explicit simulation, call `update(world.fixedDelta)` repeatedly; one large delta is subject to the catch-up limit.

`JointDrive` stiffness and damping map directly to Rapier's force/acceleration
solver motors, including native force/torque limits. A drive without a target, or
without stiffness and damping, configures no native motor. The effort term is
applied as a force pair for one substep and clamped to `maxForce` on its own. Since
that cap cannot bound the sum of both terms, a capped drive that combines gains with
effort is rejected, as is a drive's `maxVelocity`, which a constant motor force
cannot model. Revolute position targets use the
same continuous radians as `getState().position`. With nonzero stiffness, a target
must remain less than π radians from the current position at every
preparation/step boundary. Longer moves require intermediate targets; unsupported
goals throw before the solver advances. The adapter wraps accepted goals for
Rapier’s native shortest-arc motor, so holding a measured multi-turn position and
trajectories crossing ±π work without losing turn count. Velocity-only drives have
no position-target restriction. Every joint's `readJoint().angle` continues across
turns; generic `getState()` still reads wrapped Euler angles.
Independent body forces remain additive.
See the [core contracts](../../README.md#drives-and-readings) for
command lifetime, continuous angles, simulation time, mass properties, and raycasts.

## Triggers and contacts

Use `Trigger` from `@drawcall/physics` for compound overlap regions, with typed
`enter`/`exit` events and cached `overlaps(body)` / `getOverlappingBodies()` reads.
Rigid bodies emit `contactbegin`/`contactend` with `otherBody`; both APIs aggregate
shape pairs so compound shape handoffs do not create extra transitions. See the
[core example and lifecycle contract](../../README.md#triggers-and-contact-events).

Triggers detect static, kinematic, dynamic, and sleeping targets without mass or
collision response. Attached Triggers exclude their ancestor body; joint contact
suppression does not disable their detection of other links. Body and Trigger
collision-group defaults are overridden by explicit collider groups. Both masks
must permit an interaction. Raycasts exclude Trigger shapes unless
`includeTriggers: true`; narrow the hit's `kind` before reading `body` or `trigger`.

Overlap observations update only on completed fixed steps, before events and
`onAfterStep`. `update(0)` and raycasts do not populate this cached state. Reads
are sampled observations, not fresh overlap tests at final integrated transforms.
Sleeping alone does not end relationships. Trigger shapes use backend sensors as
an implementation detail; the public collider sensor flag has been removed.
