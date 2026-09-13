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

Each body requires at least one collider before backend initialization. Construction and registration never create a backend body. Complete geometry, scale, and parenting before the next `world.update(delta)` or `world.step()`. Even `update(0)` and sub-timestep updates prepare bodies, colliders, mass properties, and joints without advancing simulation time. Pending objects are also prepared before before-step callbacks; changes and objects created in those callbacks are synchronized before the solver runs.

New objects do not reset existing simulation state. Colliders follow child additions/removals, geometry changes, collider properties and materials. Body damping, gravity scale, and body type update live; `canSleep` is fixed at creation.

Body and collider scale are captured once; later scale edits throw and require disposing and recreating the affected bodies and joints. New colliders capture their scale when added. Explicit body mass stays fixed; density-derived mass and inertia follow the scaled shapes.

Joint anchors are captured on first materialization. Explicit `frame0` and `frame1` options are Three.js `Matrix4` transforms relative to their respective bodies (or world space for `body0: null`). Drives, limits and connected-contact flags update before each step. Changing joint transforms afterward does not move captured anchors. Editing explicit frame options after creation throws; dispose and create a new joint to change its anchors. Axis and distance-limit changes recreate the constraint while retaining its anchors. Disable/re-enable a joint with `joint.options.enabled`.

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

`getVelocity()` returns independent linear/angular vectors. Missing initial components default to zero; partial `setVelocity()` calls preserve the other component and copy their inputs. Before initialization, setters update authored initial velocity; afterward they update the backend's current velocity. `reset()` restores the pose and velocity captured at initialization, including construction-time setters. Later option edits and runtime operations do not rewrite that baseline.

Read the object's pose with `body.updateWorldMatrix(true, false)` followed by `body.matrixWorld`. Copy or clone it to retain a snapshot. It includes scale; `splitTransform(body.matrixWorld).pose` extracts the rigid pose. The old `world.body`, `world.joint`, and `getMatrix` APIs are removed. `teleport(matrix)` works before and after initialization and synchronously updates the object's world pose while preserving scale. Both it and `setKinematicTarget(matrix)` require a world-space rigid matrix with unit scale and no shear. After initialization the backend owns the dynamic body's pose; assigning Three.js position alone does not teleport it.

`applyImpulse`, `applyForce`, `wake`, `sleep`, and `setKinematicTarget` require a prepared Rapier body. During construction use velocity options or `setVelocity`; there is no command queue. Forces apply for one step. `onBeforeStep` and `onAfterStep` return unsubscribe functions. Bodies created inside a callback have state access immediately and become ready for simulation operations at the next preparation boundary.

`joint.getState()` exposes angle, angular velocity, position and distance. Before initialization it derives state from authored frames and body velocities; afterward it reads the backend. Disabled joints remain readable using their captured frames. Joint options can be authored before backend creation; invalid frames, disposed objects, and foreign-world access still throw.

This is a breaking API change: release the packages together under a new minor version and migrate scene consumers before deploying that release.

`body.dispose()` unregisters its physics resources and connected joints. Removing a visual from its parent does not dispose physics. `world.dispose()` disposes all registered physics objects and frees Rapier. Geometry and materials remain owned by the application.

Rapier requires equal static/dynamic friction and supports distance joints only with zero minimum distance. Unsupported authored data fails visibly.
