# @drawcall/physics-rapier

Rapier implementation of `PhysicsWorld`.

```ts
import { setupWorld } from "@drawcall/physics-rapier";
import { RigidBody, BoxCollider } from "@drawcall/physics";
import { BoxGeometry, Mesh, Vector3 } from "three";

const world = await setupWorld({ gravity: [0, -9.81, 0] });
const body = new RigidBody({ mass: 1 });
body.add(new Mesh(new BoxGeometry(1, 1, 1)));
scene.add(body);
world.update(deltaSeconds);
```

`setupWorld()` initializes Rapier and installs the new default world. Objects capture their world when constructed. A later setup affects only new objects. Joints use their connected bodies' world.

`setupWorld({ solverIterations: 16 })` increases constraint solver precision for demanding joint chains, such as vehicle wheel assemblies. The value must be a positive integer; omitting it preserves Rapier’s default. Higher values cost more CPU time.

Dynamic bodies require collider-derived mass/inertia or explicit positive mass and inertia; static and kinematic bodies may be colliderless. Construction and registration never create a backend body. Complete geometry, scale, and parenting before the next `world.update(delta)`. Even `update(0)` and sub-timestep updates prepare bodies, colliders, mass properties, and joints without advancing simulation time. Pending objects are also prepared before before-step callbacks; changes and objects created in those callbacks are synchronized before the solver runs.

New objects do not reset existing simulation state. Colliders follow child additions/removals, geometry changes, collider properties and materials. Body damping and gravity scale methods update live; body type and `canSleep` are fixed at creation.

Body and collider scale are captured once; later scale edits throw and require disposing and recreating the affected bodies and joints. New colliders capture their scale when added. Explicit body mass stays fixed; density-derived mass and inertia follow the scaled shapes.

Joint anchors are captured on first materialization. Explicit `frame0` and `frame1` options are Three.js `Matrix4` transforms relative to their respective bodies (or world space for `body0: null`). Limit and connected-contact setters update before each step. Changing joint transforms afterward does not move captured anchors. Editing explicit frame options after creation throws; dispose and create a new joint to change its anchors. Distance-limit changes recreate the constraint while retaining its anchors. Disable/re-enable a joint with `joint.setEnabled(value)`.

Use methods directly on the objects:

```ts
const body = new RigidBody({ mass: 2 });
body.setVelocity({ linear: new Vector3(2, 0, 0) });
body.getVelocity(); // available immediately, before colliders or backend
body.add(new BoxCollider());
scene.add(body);
world.update(0); // completed assembly is now ready for simulation operations
body.applyImpulse(new Vector3(1, 0, 0));
```

`getVelocity()` returns independent world-space linear/angular vectors; linear velocity is measured at the center of mass. Missing initial components default to zero; partial `setVelocity()` calls preserve the other component and copy their inputs. Before initialization, setters update authored initial velocity; afterward they update the backend's current velocity. `reset()` restores the pose and velocity captured at initialization, including construction-time setters. Later runtime operations do not rewrite that baseline.

Read the object's pose through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning, so observation callbacks after a step need no refresh. Call `body.updateWorldMatrix(true, false)` only when reading immediately after direct authoring or hierarchy changes. Copy or clone it to retain a snapshot. It includes scale; `splitTransform(body.matrixWorld).pose` extracts the rigid pose. The old `world.body`, `world.joint`, and `getMatrix` APIs are removed. `teleport(matrix)` works before and after initialization and synchronously updates the object's world pose while preserving world scale. Under a nonuniformly scaled static parent, the local scale may change to compensate; transforms requiring shear are rejected before the object or backend is moved. Both it and `setKinematicTarget(matrix)` require a world-space rigid matrix with unit scale and no shear. After initialization the backend owns the dynamic body's pose; assigning Three.js position alone does not teleport it.

`applyImpulse`, `applyForce`, `wake`, `sleep`, and `setKinematicTarget` require a prepared Rapier body. During construction use `setVelocity`; there is no command queue. Forces apply for one step. `onBeforeStep` and `onAfterStep` return unsubscribe functions. Bodies created inside a callback have authored velocity access immediately and become ready for simulation operations at the next preparation boundary.

Axis `joint.getState()` exposes `{ position, velocity }`; other joint types expose their constraint-specific state. Before initialization it derives state only from known authored inputs. A rotating slider without explicit COM needs prepared bodies: finish assembly and call `world.update(0)` or let the next update prepare them. It throws while required mass data is unavailable. Once prepared, Rapier computes the collider-derived mass properties and supplies anchor velocities through `velocityAtPoint`; core never calculates mass properties. Disabled joints remain readable using their captured frames. Mutable joint settings use methods before and after backend creation; invalid frames, disposed objects, and foreign-world access still throw.

This is a breaking API change: release the packages together under a new minor version and migrate scene consumers before deploying that release.

`body.dispose()` unregisters its physics resources and connected joints. Removing a visual from its parent does not dispose physics. `world.dispose()` disposes all registered physics objects and frees Rapier. Geometry and materials remain owned by the application.

Rapier requires equal static/dynamic friction and supports distance joints only with zero minimum distance. Unsupported authored data fails visibly.

`update(delta)` accumulates elapsed seconds and runs fixed simulation steps, up to the configured `maxSubsteps` catch-up limit. `update(0)` prepares without advancing time. For explicit simulation, call `update(world.fixedDelta)` repeatedly; one large delta is subject to the catch-up limit.

`world.time` advances by `fixedDelta` per completed solver step, before after-step
observers run. Even if an after-step observer throws, that completed interval is
consumed and cannot be replayed. Catch-up time beyond `maxSubsteps` is discarded,
not counted as simulated time. Reset restores zero.

Revolute/prismatic `setEffort` applies one substep of torque/force with equal
reaction on the connected dynamic body. Last call wins, zero cancels, and pending
effort survives no-step updates. Disable/reset/dispose clear commands. Independent
body forces remain independent. Controllers should submit effort in `onBeforeStep`.
The car example implements suspension, steering, and propulsion locally.

Revolute measurements track continuous turns at each substep. Angular movement
must stay below π radians per substep; orientation samples cannot distinguish
larger jumps. Teleports rebase without adding traveled revolutions; reset restores
the initialized coordinate. Prismatic velocity includes moving anchors and axis.

`raycast(origin, direction, maxDistance, options)` queries prepared state without
stepping. It returns the closest exit-surface hit (including for inside origins),
with source identity, or null. Options support `collisionGroups`, `includeSensors`
(default false), and `excludeBodies`. Pending construction stays absent until
`update(0)` or another update prepares it. Teleports update queries immediately.

Explicit `mass`, `centerOfMass`, `diagonalInertia`, and `principalAxes` are immutable
body-local properties. Complete specifications override collider contributions;
partial specifications infer missing properties from colliders. Explicit inertia
is supplied in physical kg·m² and is not rescaled with visual geometry. See the
core README for units, validity requirements, and the breaking migration table.
