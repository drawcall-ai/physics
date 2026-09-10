# @drawcall/physics-rapier

Rapier implementation of `PhysicsWorld`.

```ts
import { setupWorld } from "@drawcall/physics-rapier";
import { RigidBody } from "@drawcall/physics";
import { BoxGeometry, Mesh } from "three";

const world = await setupWorld({ gravity: [0, -9.81, 0] });
const body = new RigidBody({ mass: 1 });
body.add(new Mesh(new BoxGeometry(1, 1, 1)));
scene.add(body);
world.update(deltaSeconds);
```

`setupWorld()` initializes Rapier and installs the new default world. Objects capture their world when constructed. A later setup affects only new objects. Joints use their connected bodies' world.

Each body requires at least one collider before materialization. Registration is deferred until stepping or requesting body/joint controls. Configure geometry and transforms before those calls. New bodies and joints can be created at any time, including before-step callbacks, without resetting existing simulation state. Colliders follow child additions/removals, geometry changes, collider properties and materials. Body damping, gravity scale, and body type update live; `canSleep` is fixed at creation. Velocity options specify initial/reset velocity; controls change current velocity.

Joint anchors are captured on first materialization. Explicit `frame0` and `frame1` options are Three.js `Matrix4` transforms relative to their respective bodies (or world space for `body0: null`). Drives, limits and connected-contact flags update before each step. Changing joint transforms afterward does not move captured anchors. Editing explicit frame options after creation throws; dispose and create a new joint to change its anchors. Axis and distance-limit changes recreate the constraint while retaining its anchors. Disable/re-enable a joint with `joint.options.enabled`.

`world.body(body)` exposes `getMatrix(target?)`, velocity, teleport, impulse, force, sleep and kinematic controls. `teleport(matrix)` and `setKinematicTarget(matrix)` accept a Three.js `Matrix4` in world coordinates; transforms must have unit scale and no shear. `world.joint(joint).getState()` exposes angle, angular velocity, position and distance. `onBeforeStep` and `onAfterStep` return unsubscribe functions. Forces apply for one step. `reset()` restores each registered body's initial pose and velocity.

`body.dispose()` unregisters its physics resources and connected joints. Removing a visual from its parent does not dispose physics. `world.dispose()` disposes all registered physics objects and frees Rapier. Geometry and materials remain owned by the application.

Rapier requires equal static/dynamic friction and supports distance joints only with zero minimum distance. Unsupported authored data fails visibly.
