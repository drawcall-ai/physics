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

Constructor options are copied and typed readonly: body type/mass, collider dimensions,
joint bodies/frames/limits/dofs, and drive gains. `options` carries the resolved
defaults. Recreate objects to change them.
Mutable settings use methods: `setVelocity`, `setLinearDamping`, `setAngularDamping`,
`setGravityScale`, `setMaterial`, `setEnabled`, `setCollideConnected`, `setDrive`,
and a drive's `setTarget`.
Private state and readonly configuration use TypeScript; numeric physics and
external-input constraints are checked at runtime.

`body.dispose()` releases its resources and connected joints. Removing a body from
its Three.js parent does not dispose it. `world.dispose()` releases all its objects
and clears the default only if that world is still the default. A later
`setupWorld()` affects new objects; existing bodies retain their original world.

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

Explicit colliders use `setSensor(true)`, `setMaterial(...)`, and
`setCollisionGroups({ membership, filter })` with unsigned 16-bit masks.
Dimensions use constructor options, e.g. `new BoxCollider({ size: [1, 2, 3] })`.
Mesh geometry uses `setGeometry`; its approximation remains immutable. Adapters
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

### State access and static previews

`body.getVelocity()`, `body.setVelocity({ linear, angular })`, `body.teleport(pose)`, and authored joint reads work during scene construction, including while a host stages objects outside world registration. Velocity defaults to zero. Input and output vectors are independent copies. Read transforms through `body.matrixWorld`. Physics writeback and teleportation synchronize it before returning; observation after a step needs no refresh. After direct authoring or hierarchy changes, call `body.updateWorldMatrix(true, false)` if reading immediately. That matrix includes scale; `splitTransform(body.matrixWorld).pose` gives a rigid pose for teleportation.

`AuthoringWorld` permits a static preview to run one scene callback without a simulation backend. Velocity is stored authored state; teleportation updates the object immediately. Valid forces, impulses, kinematic targets, and sleep/wake calls are explicitly inert; they do not move the object or alter velocity. Step observers can register/unsubscribe but never run. Invalid arguments and disposed/foreign objects still fail. Calling `update` or `reset` on an authoring world still throws because it cannot simulate.

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
AuthoringWorld requires explicit mass properties for this read and otherwise throws.
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
null: distance, world point/normal, body, and source collider/mesh. Directions are
normalized; distances are meters. Options include `collisionGroups`, `excludeBodies`,
and `includeSensors` (default false). Inside-origin rays return the exit surface.
Queries include the current authored scene before the first update, without advancing time.
AuthoringWorld has no raycasts. Release all three packages together for this breaking API.
