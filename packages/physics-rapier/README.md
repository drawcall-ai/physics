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

Dynamic bodies require collider-derived mass/inertia or a complete explicit mass specification; static and kinematic bodies may be colliderless. Construction and registration never create a backend body. Complete geometry, scale, and parenting before the next `world.update(delta)`. Even `update(0)` and sub-timestep updates prepare bodies, colliders, mass properties, and joints without advancing simulation time. Pending objects are also prepared before before-step callbacks; changes and objects created in those callbacks are synchronized before the solver runs.

New objects do not reset existing simulation state. Colliders follow child additions/removals, geometry changes, collider properties and materials. Body damping, gravity scale, and `setType("dynamic" | "static" | "kinematic")` update live; `canSleep` is fixed at creation. Bodies default to dynamic. An actual type transition clears velocity, body forces, and any queued kinematic target. Calling `setType()` with the current type leaves motion unchanged. A later velocity or kinematic-target command applies to the new type. Unsupported transitions, such as a triangle-mesh body becoming dynamic, throw and preserve the previous type.

Body and collider scale are captured once; later scale edits throw and require disposing and recreating the affected bodies and joints. New colliders capture their scale when added. Explicit body mass stays fixed; density-derived mass and inertia follow the scaled shapes.

Joint anchors are captured on first materialization. Explicit `frame0` and `frame1` options are Three.js `Matrix4` transforms relative to their respective bodies (or world space for `body0: null`). Physical limits are copied immutable constructor options. Connected-contact settings update through `setCollideConnected()`. Changing joint transforms afterward does not move captured anchors. Dispose and create a new joint to change its anchors or limits. Disable/re-enable a joint with `joint.setEnabled(value)`.

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

## Joint motors and direct effort

`JointMotor` maps to Rapier's native solver motor; targets persist across substeps.

```ts
const motor = new JointMotor({
  joint: hinge,
  stiffness: 100,
  damping: 10,
  maxForce: 20,
});
motor.setTarget({ position: 0.5 });
motor.setEnabled(false);
```

Import `JointMotor` from `@drawcall/physics`. Motor configuration is immutable;
recreate the motor to change gains, maximum force, model, or connected joint.
A new motor is enabled but has no target and exerts no actuation. Each
`setTarget()` replaces the previous target, filling omitted position or velocity
with zero. Configured stiffness and damping both remain active. Use zero
stiffness for a pure velocity motor; zero velocity with damping produces braking.
Disabling or disposing the motor removes actuation and preserves the joint.
The default `model: "force"` and optional `model: "acceleration"` use Rapier's
corresponding formulations. `maxForce` bounds force in N for sliders and torque in
N·m for hinges; Rapier handles timestep conversion. Absent `maxForce` is unbounded.
Native motors drive the car's suspension, steering, and axles.

Revolute/prismatic `setEffort()` requires a prepared joint, just like body force
operations. Finish assembly and call `world.update(0)`, or submit commands from
`onBeforeStep`, which runs after existing assemblies are prepared. Newly created
joints inside that callback must wait for a preparation boundary before effort.
A command applies torque/force for one substep with equal reaction on the connected
dynamic body. Last call wins, zero cancels, and pending effort survives no-step
updates. Disable/reset/dispose clear commands. Independent body forces remain
independent. Zero cancellation on an unprepared joint is harmless.

Disable an active motor before direct effort control. A nonzero effort with an
active motor throws, including when a motor is activated after effort was set;
no part of that substep is simulated. Cancel the effort with `setEffort(0)` or
disable the motor explicitly. Combined motor and feed-forward effort is not
supported.

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
body-local properties. Specify either total `mass` alone or `mass`, `centerOfMass`, and
`diagonalInertia` together. `principalAxes` is optional for the complete form and
defaults to identity. Other partial specifications are rejected. Complete
specifications override collider mass and inertia even after geometry edits and
permit valid colliderless bodies. Mass-only bodies retain collider-derived COM
and inertia, with density-weighted masses normalized to the specified total;
zero-density colliders use uniform density for this inference. Explicit inertia
is supplied in physical kg·m² and is not rescaled with visual geometry. See the
core README for units, validity requirements, and the breaking migration table.
