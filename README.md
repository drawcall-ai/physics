# @drawcall/physics

Physics scene objects for Three.js. Bodies, colliders, and joints are ordinary
`Object3D`s that you build, clone, import, and export without an engine. A backend
package simulates them.

| Package                                                                                                       | Purpose                                |
| ------------------------------------------------------------------------------------------------------------- | -------------------------------------- |
| `@drawcall/physics`                                                                                           | Scene objects and shared contracts     |
| [`@drawcall/physics-rapier`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-rapier#readme) | Rapier simulation                      |
| [`@drawcall/physics-mujoco`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-mujoco#readme) | MuJoCo WASM simulation (browser, Node) |
| [`@drawcall/physics-usd`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-usd#readme)       | USD Physics import and export          |

## Quick start

```sh
npm install three @drawcall/physics @drawcall/physics-rapier
```

```ts
import { BoxGeometry, Mesh, Scene } from "three";
import { RevoluteJoint, RigidBody } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-rapier";

const scene = new Scene();
const frame = new RigidBody({ bodyType: "static" });
frame.add(new Mesh(new BoxGeometry(0.1, 2.2, 0.15)));

const door = new RigidBody({ mass: 20 });
door.position.x = 0.55;
door.add(new Mesh(new BoxGeometry(0.98, 2, 0.06)));

const hinge = new RevoluteJoint({
  body0: frame,
  body1: door,
  limits: [0, Math.PI / 2],
});
hinge.position.x = 0.06;
scene.add(frame, door, hinge);

const world = await buildWorld({ scene, gravity: [0, -9.81, 0] });
world.update(1 / 60); // from your render loop, elapsed seconds
world.dispose(); // when done
```

For MuJoCo, import `buildWorld` from `@drawcall/physics-mujoco` instead.

## Worlds

`await buildWorld(options)` builds a world for the bodies, joints, and triggers
under `options.scene` and prepares them, without advancing time. `scene` may be any
`Object3D` subtree but is normally the three.js `Scene`. Options:

| Option             | Default         | Meaning                                           |
| ------------------ | --------------- | ------------------------------------------------- |
| `scene`            | required        | The `Object3D` whose descendants are simulated    |
| `gravity`          | `[0, -9.81, 0]` | m/s²                                              |
| `fixedDelta`       | `1 / 60`        | Fixed step in seconds                             |
| `maxSubsteps`      | `5`             | Steps per `update`; extra elapsed time is dropped |
| `solverIterations` | backend default | Constraint solver iterations per step             |

Before every `update`, step, and query, the world traverses `scene`: objects added
under it join, objects removed from it leave. Joining prepares the object in the
backend and sets the read-only `object.world`, which is undefined again as soon as
the scene no longer holds the object.

- A scene has at most one live world; building a second throws until the first is
  disposed.
- An object belongs to at most one world. Moved under another world's scene, it
  leaves its old world as the new one takes it; under the scenes of two worlds at
  once, it throws.
- A joint under `scene` whose body is not under `scene` throws on the next update.
- Joining captures scale and joint anchors, so finish them before the object joins.
  Recreate the object to change them later.
- A failed build disposes the world and leaves the scene as authored.

| Member                                       | Description                                                                      |
| -------------------------------------------- | -------------------------------------------------------------------------------- |
| `update(delta)`                              | Advances by `delta` seconds in fixed steps                                       |
| `reset()`                                    | Syncs, then returns members to the state captured when they joined; `time` = 0   |
| `raycast(...)`                               | Closest hit, see [Raycasts](#raycasts)                                           |
| `decompose(root)`                            | Prepares convex parts, see [Triangle meshes](#triangle-meshes-and-decomposition) |
| `onBeforeStep(cb)`                           | Called with `fixedDelta` before each step; returns an unsubscribe function       |
| `onAfterStep(cb)`                            | Called after each step and its events; returns an unsubscribe function           |
| `time`                                       | Simulated seconds of completed steps                                             |
| `scene`, `gravity`, `fixedDelta`, `disposed` | Read-only state                                                                  |
| `dispose()`                                  | Frees native resources; members leave the world                                  |

`update` and `reset` throw when called from a step callback or event listener. A
callback added during a step first runs at the next step. A step that throws ends
the update; the steps it completed count, and the rest of its time is dropped.

### Simulation commands

The simulation reads and commands on objects need a world: `getVelocity`,
`setVelocity`, `teleport`, `setKinematicTarget`, `applyImpulse`, `applyForce`,
`wake`, `sleep`, `trigger.overlaps`, `getOverlappingBodies`, and joint `getState`. They
go to the world the object joined while that world's scene still holds it, or else to
the built world whose scene holds it, which the object joins right then. Anywhere else,
including while its world is still building, they throw: add the object under a built
world's scene first.

An object joins at the world's next `update`, step, or query, or at the first command
that reaches it, so a command right after adding an object works in any loop order
and inside step callbacks. Before-step callbacks see the objects added since the last
step. A body joins with its join pose and its `velocity` option, which `reset()`
restores; a body that leaves and rejoins starts from them again. If joining fails,
the object stays unjoined, the command throws, and the object joins afresh once the
error is fixed. Until the object is fixed or removed, `update`, queries, and commands
on objects that have not joined yet throw too: a failed join is not isolated to its
object. A command on an object moved under another world's scene syncs the world it
left, so it can throw the join errors of either world's other objects.

The world methods `getVelocity`, `setVelocity`, `teleport`, `setKinematicTarget`,
`applyImpulse`, `applyForce`, `wake`, `sleep`, and `getOverlappingBodies` take the
object as their first argument. They only act on objects under that world's scene
and throw for any other.

Velocity, forces, and impulses require a dynamic body. `teleport` moves every
dynamic body jointed to the body along with it, following the joints that have
joined (it syncs first). An assembly jointed to the world or
a static or kinematic base can only move within those joints; backends differ in
how they handle that.

Read poses from `body.matrixWorld`; steps and `teleport` keep it current. It includes
scale; `splitTransform(matrix).pose` returns the rigid part.

## Objects

Constructor options are immutable and exposed as `options`. Recreate an object to
change them. Mutable settings use `set*` methods.

### RigidBody

`RigidBody extends Group`.

```ts
new RigidBody({
  bodyType: "dynamic", // or "static", "kinematic"
  colliders: "auto", // "box", "convexHull", "trimesh", or false
  canSleep: true,
  mass: 2, // optional
  velocity: { linear: [0, 0, 2], angular: [0, 1, 0] }, // optional, rad/s
});
```

- Mass: omit for density-derived mass, give `mass` alone to scale it, or give
  `mass`, `centerOfMass`, `diagonalInertia`, and optional `principalAxes`
  (`[x, y, z, w]`) in body-local units. Explicit values are not scaled by
  transforms. A dynamic body without colliders needs complete mass properties.
- `velocity` is the velocity a dynamic body joins a world with and `reset()` returns
  it to. Other body types throw for any `velocity`, as do unknown keys in it and
  parts that are not three finite numbers.
- Unknown option keys throw.
- Methods: `setVelocity`, `setLinearDamping`, `setAngularDamping` (rates in 1/s),
  `setGravityScale`, `setMaterial`, `setCollisionGroups`, plus the commands above.
- Events: `contactbegin` and `contactend` with `{ otherBody }`.

### Colliders

Without explicit colliders a body generates one per visual mesh, per its
`colliders` option. `"auto"` keeps unchanged primitive geometries as primitives;
other geometry becomes a convex hull on moving bodies and triangles on static ones.

Explicit colliders disable generation on their body. Their transforms place the shape
relative to the body:

- `BoxCollider({ size })` takes a full size.
- `SphereCollider({ radius })`.
- `CapsuleCollider({ radius, height })` and `CylinderCollider({ radius, height })`
  extend along Y; capsule `height` excludes the caps.
- `MeshCollider({ approximation })` with `"convexHull"` (default) or `"trimesh"`;
  assign geometry with `setGeometry`.

Colliders have `setMaterial` and `setCollisionGroups`, which replace the body's
defaults. `body.getColliders()` returns the explicit or generated colliders, and
`body.getMaterial(collider)` the resolved material.

Materials are plain objects: `{ staticFriction, dynamicFriction, restitution, density }`;
omitted values use defaults.

Collision groups are `{ membership, filter }` 16-bit masks. A pair collides when
`(a.membership & b.filter) !== 0 && (b.membership & a.filter) !== 0`.

### Triangle meshes and decomposition

Where a backend cannot collide a `trimesh` as triangles (moving bodies in Rapier,
everything in MuJoCo), the world decomposes it into convex parts with CoACD when it is
built. For a mesh added or edited after the build, `await world.decompose(object)`
before it joins; joining without parts throws and says so. CoACD needs closed,
consistently wound surfaces; open surfaces fail, so use `"convexHull"` where one hull
is enough.

```ts
await world.decompose(asset); // only decomposes what this backend needs
scene.add(asset);
```

CoACD is embedded as a lazily imported module with inline WASM, so no asset hosting or
bundler configuration is needed. It runs on the calling thread. In Node 20.16+, parts
are cached in `node_modules/.cache/@drawcall/physics` under the working directory;
delete it to decompose afresh.

Geometry edits are seen the way three.js renderers see them: replace the position or
index attribute, or set `needsUpdate = true` after an in-place edit.

### Scale

Boxes and meshes support positive nonuniform scale. Spheres and capsules need uniform
scale; cylinders need equal X/Z scale. Moving bodies need uniform ancestor scale.
`resolveCollider(body, collider)` returns the body-local matrix and a shape with
world scale baked in.

Rejected: zero, negative, or sheared transforms, nested bodies, partial draw ranges,
and automatic colliders on instanced, skinned, or morphed meshes.

### Joints

`FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `SphericalJoint`, `DistanceJoint`,
and `GenericJoint` extend `Object3D` and connect `body0` (base, or `null` for the
world) to `body1`.

- The joint's world placement defines both anchors. Alternatively pass both `frame0`
  and `frame1` as body-local `Matrix4`s (world space for a `null` `body0`).
- Revolute and prismatic joints take `axis` (`"X"`, `"Y"` default, `"Z"`) and optional
  `limits`.
- Distance joints require `limits: [min, max]`; `Infinity` leaves the maximum free.
- Generic joints take `dofs`, mapping `transX`…`rotZ` to `"locked"` (default),
  `"free"`, or `[min, max]` in frame 0.
- `setEnabled(bool)`, `setCollideConnected(bool)`, `getFrame(index, target)`.

`getState()` returns:

| Joint               | State                                                              |
| ------------------- | ------------------------------------------------------------------ |
| Revolute, prismatic | `{ position, velocity }`; revolute angles count turns              |
| Spherical           | `{ rotation, angularVelocity }` of frame 1 relative to frame 0     |
| Distance            | `{ distance, velocity }`                                           |
| Generic             | `getState(axis)`: `{ position, velocity }`, rotations as XYZ Euler |

Revolute angles track turns per step, so a joint must turn less than π per step.

### Drives

`JointDrive` is a plain class that applies
`stiffness · (position − q) + damping · (velocity − q̇) + effort`, capped by
`maxForce`, to one joint coordinate.

```ts
import { JointDrive } from "@drawcall/physics";

const drive = new JointDrive({ stiffness: 100, damping: 10, maxForce: 20 });
hinge.setDrive(drive);
drive.setTarget({ position: 0.5 }); // servo
drive.setTarget({ velocity: 2 }); // motor (omitted terms become 0)
drive.setTarget(undefined); // passive
hinge.setDrive(undefined); // detach
```

- Revolute, prismatic, and distance joints hold one drive; generic joints hold one per
  axis via `setDrive(axis, drive)`.
- A position target needs stiffness; a velocity target needs damping.
- `model: "acceleration"` scales gains by the driven mass; default `"force"`.
- `maxVelocity` (needs `maxForce`) is the motor's no-load speed: it adds damping of
  `maxForce / maxVelocity`.
- Subclasses survive cloning, so a drive can carry application data.

A drive with a constant target is a spring. To drag a dynamic body while contacts
still stop it, drive a free generic joint from a kinematic hand:

```ts
import { Matrix4, type Object3D, type Vector3 } from "three";
import { GenericJoint, JointDrive, RigidBody } from "@drawcall/physics";

function grab(scene: Object3D, body: RigidBody, point: Vector3) {
  const hand = new RigidBody({ bodyType: "kinematic", colliders: false });
  hand.position.copy(point);
  const hold = new GenericJoint({
    body0: hand,
    body1: body,
    frame0: new Matrix4(),
    frame1: new Matrix4().makeTranslation(body.worldToLocal(point.clone())),
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
  hand.add(hold);
  scene.add(hand);
  return hand; // hand.setKinematicTarget(pose) each frame; hand.removeFromParent() releases
}
```

### Triggers

`Trigger extends Group` detects overlap with the colliders added beneath it. It has no
mass or collision response, no automatic colliders, and may sit under a body to
follow it (it ignores that body).

```ts
import { BoxCollider, Trigger } from "@drawcall/physics";

const goal = new Trigger();
goal.add(new BoxCollider({ size: [0.1, 0.1, 0.1] }));
scene.add(goal);
goal.addEventListener("enter", ({ body }) => console.log("entered", body));
goal.addEventListener("exit", ({ body }) => console.log("left", body));
```

- `enter`/`exit` fire per rigid body; body `contactbegin`/`contactend` fire per body
  pair on both bodies.
- `overlaps(body)` and `getOverlappingBodies()` read the last completed step.
- Events dispatch after the step, before `onAfterStep`. Removing an object from the
  scene ends its overlaps and contacts; `reset()` and `dispose()` clear them silently.
- A throwing listener does not stop delivery: every queued event, including those
  other listeners queue, is delivered before the errors are rethrown.
- Triggers do not detect other triggers.

### Raycasts

```ts
const hit = world.raycast(origin, direction, maxDistance, {
  collisionGroups, // same mutual rule as contacts
  excludeBodies: [player],
  includeTriggers: false, // default
});
if (hit?.kind === "body")
  console.log(hit.body, hit.distance, hit.point, hit.normal);
```

Hits carry `distance`, world `point` and `normal`, and the hit `object` (the collider,
or the visual mesh an automatic collider stands for), plus
`body` or `trigger` by `kind`. Rays starting inside a shape return the exit surface.

### Cloning

To clone anything holding joints, use `clone(root)`, which reconnects joints to the
cloned bodies (like `SkeletonUtils.clone`). `joint.clone()`, and so a three.js
`clone()` or `copy()` of a hierarchy holding a joint, throws, since the copy would
still connect the original bodies. Geometry and materials stay shared. Clones are
simulated once added under a world's scene.

`RigidBody`, `Joint`, `Trigger`, and `Collider` carry `isPhysicsObject = true`, like
three's `isMesh`, so a host can find physics in a subtree without importing a
backend.

## Conventions

These hold for every backend. A backend rejects what it cannot honor rather than
approximating.

- SI units and radians throughout.
- Damping is a rate in 1/s, independent of mass.
- Linear velocity is at the center of mass; angular velocity and force/impulse
  `point`s are world space.
- `body0` is a joint's base side. Reduced-coordinate backends build their tree from
  it, so prefer authoring a tree.
- Friction/restitution combine rules and contact softness are backend-defined.
- Immutable configuration uses tuples; runtime state uses three.js vectors,
  quaternions, and matrices.

## Writing a backend

`@drawcall/physics/backend` holds the adapter API; scene code never needs it. A backend:

1. Extends `PhysicsWorld`, passing `super(options, decomposes)`, where `decomposes(body,
geometry)` selects the triangle meshes it collides as convex parts.
2. Implements the protected hooks:

   | Hook                                                   | Does                                                     |
   | ------------------------------------------------------ | -------------------------------------------------------- |
   | `add(object)`, `remove(object)`                        | An object joins (joints after their bodies) or leaves    |
   | `prepare()`                                            | Reconciles the members' authored changes                 |
   | `step()`                                               | Advances one fixed step and writes poses back            |
   | `restore()`, `free()`                                  | Returns members to their join state; frees native memory |
   | `cast(origin, unitDirection, maxDistance, options)`    | Closest raycast hit                                      |
   | `readVelocity(body)`, `writeVelocity(body, value)`     | Live velocity                                            |
   | `writePoses(bodies)`                                   | Adopts the scene poses a teleport wrote for these bodies |
   | `writeTarget(body, pose)`                              | Kinematic target for the next step                       |
   | `writeImpulse(body, impulse, point?)`, `writeForce(…)` | Impulse now; force over the next step                    |
   | `writeSleeping(body, sleeping)`                        | Sleep or wake                                            |
   | `jointReading(joint)`                                  | The joint's reading, usually `JointBinding.read(motion)` |

   The base class syncs membership with the scene, sets `object.world`, runs the fixed
   clock and callbacks, dispatches events, and validates every command: hooks are
   called only for joined members, with finite vectors, rigid poses, a unit ray
   direction, valid collision groups, and the body type the command needs. A body
   joins with `initialVelocity(body)`, which `restore()` returns it to along with its
   join pose.

3. Exports `buildWorld(options)`: await the engine, then return
   `build(new MyWorld(...))`, which decomposes the scene's meshes and joins its objects.

`prepare()` reconciles authored changes. Bodies, joints, triggers, and colliders count
setting changes in `version`; `geometryVersion(geometry)` tracks geometry edits.
`step()` should write poses back with `setWorldPose` and report overlaps and contacts
through `this.interactions.replace(...)`.

Helpers: `initialVelocity`; `JointBinding` (a joint's captured frames and
continuous angle: `read(motion)`, pose-only `pose()`), `wrapAngle`, `dofState`,
`dofPosition` (joint readings); `treeJoint`, `unconstrained` (joint graphs);
`convexParts`; `lockScale` (scale fixed on joining); `resolveCollisionGroups`,
`axisVector`; `cleanup` and `rollback` (error aggregation).

The Rapier package's tests are the executable contract. Start a new backend from them
and keep each engine limitation as a test next to its rejection, as
[`rapier.test.ts`](https://github.com/drawcall-ai/physics/blob/main/packages/physics-rapier/test/rapier.test.ts)
does.

## Examples

In a clone of the [repository](https://github.com/drawcall-ai/physics):

```sh
pnpm --filter @drawcall/example-ragdoll dev # articulated ragdoll, pointer grab
pnpm --filter @drawcall/example-car dev     # powered car with suspension
pnpm --filter @drawcall/example-scale dev   # scaled colliders and decomposition
```

Each example has a backend dropdown; switching restarts the simulation.

## Development

```sh
pnpm install
pnpm --filter @drawcall/physics build:coacd
pnpm check
```

`build:coacd` downloads pinned CoACD, CDT, and Chitin sources, installs Emscripten
5.0.2 if needed, and writes the gitignored `packages/physics/src/colliders/coacd.ts`
(JavaScript with embedded WASM) and `packages/physics/generated/coacd/` (source archive
and license notices). It needs Bash, curl, tar, CMake, Node.js, and Python 3. CI runs it
before checks and publishing, so npm consumers need no compiler.
