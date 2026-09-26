# @drawcall/physics-mujoco

MuJoCo WASM implementation of `PhysicsWorld`, using the official
[`@mujoco/mujoco`](https://github.com/google-deepmind/mujoco/tree/main/wasm) bindings.
The single-threaded engine works in Node.js and browsers without cross-origin
isolation headers.

```sh
npm install @drawcall/physics @drawcall/physics-mujoco three
```

In Node.js the engine loads its packaged WASM automatically:

```ts
import { buildWorld } from "@drawcall/physics-mujoco";
import { RigidBody, BoxCollider } from "@drawcall/physics";

const box = new RigidBody({ mass: 1 });
box.position.y = 2;
box.add(new BoxCollider());
const world = await buildWorld({ fixedDelta: 1 / 120 });
world.update(world.fixedDelta);
console.log(box.position.y);
world.dispose();
```

With Vite, import the WASM as an asset and pass its URL. Add `@mujoco/mujoco`
as a direct dependency when importing this asset from application code:

```ts
import { buildWorld } from "@drawcall/physics-mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";

const world = await buildWorld({ wasmUrl, solverIterations: 50 });
```

`frictionCone` selects MuJoCo's friction model. The default `"pyramidal"` is what MuJoCo
ships and proved more robust for kinematic contact at small steps; `"elliptic"` models
friction faithfully and is the cone `frictionImpedanceRatio` is defined for.
`frictionImpedanceRatio` is MuJoCo's `impratio`: how stiff friction constraints are
relative to normal ones. At the default `1` a resting or grasped object still creeps,
because soft friction trades slip for force. Raising it converges on Coulomb friction
without changing the limit at which contacts start to slide; grasping needs around `50`
with elliptic cones, and a ratio above `1` without them is rejected.

A drive's `maxVelocity` is honoured here as the motor's back-EMF: the joint is damped by
`maxForce / maxVelocity`, so a saturated drive settles at its rated speed. The damping sits on
the joint rather than the actuator, both because back-EMF resists motion whenever the motor is
connected and because MuJoCo integrates joint damping implicitly, which a force limit that
chased the measured speed did not survive. Distance joints and free generic joints,
which MuJoCo drives by generalized force, brake by the same damping.

Other bundlers must serve `mujoco.wasm` and supply its URL through `wasmUrl`.
The examples demonstrate both development and production asset loading.
`buildWorld` prepares registered colliders, attaches to the single physics registry,
and compiles the initial model without advancing time. MuJoCo modules are shared;
each world owns and frees its model and simulation data. Build after authoring the
initial scene to enable mesh optimization. Building first is also supported.

## Simulation and scene edits

`update` accumulates elapsed seconds into fixed steps, capped by `maxSubsteps`.
`update(0)` compiles pending geometry without advancing time. Complete geometry,
parenting, and initial transforms before preparing the world. Shape and body
scale and joint anchors are captured at first preparation; recreate objects to
change their scale or anchors. `reset` restores captured poses and velocities.
Callbacks, disposal, forces, impulses, world-space teleports, kinematic targets,
raycasts, triggers, collision groups, and contact transitions use the core API.

MuJoCo compiles a whole articulated model. Adding/removing bodies, changing
colliders, or attaching/disabling joints rebuilds it transactionally and preserves
current poses and joint velocities. Drive targets update native actuators without
recompilation. Rebuilds are more expensive than Rapier's incremental edits.
Raycasts use a temporary model so a read never captures permanent authoring state.

Contacts go through MuJoCo's own broadphase. Each distinct collision group becomes
`contype`/`conaffinity` bits that collide exactly as the groups do, so model size
and step cost grow with the number of shapes, not with their pairs; MuJoCo's 32
bits cover any practical set of groups. Friction combines as the larger of the
two, MuJoCo's rule. Bodies held by a `FixedJoint` share one MuJoCo body, so
`collideConnected` is an error there.

The native `discrete` integrator treats servo stiffness and damping implicitly
alongside contacts. Saturated actuators lose these implicit derivatives, so
force-limited servos still need gains and target rates appropriate to the timestep.
Hinge and slider drives use native actuators with force limits. Completely
free generic joints, including the ragdoll hand, and distance drives use scalar
implicit springs projected through the native mass matrix. Their effective inertia
includes both connected bodies, angular inertia, and anchor lever arms.
Frame-relative translation applies equal opposite forces at the driven anchor,
including the reaction torque from frame rotation. Acceleration drives scale gains
by the coordinate's effective inertia. Contacts remain the native solver's
responsibility. Distance limits use spatial tendons. Revolute readings track full turns.

## Engine differences

- Constrained joints must form a directed tree: one parent joint per dynamic
  `body1`, with no cycles. Distance tendons and fully free generic drive joints may
  connect different branches. Locked frame coordinates must be aligned before
  creating or re-enabling a constraint. Unsupported graphs and misaligned locked
  frames throw.
- Teleport and velocity setters after compilation operate on free dynamic roots,
  static bodies, and kinematic targets. Use drives and impulses for articulated
  links. Teleporting a free root moves its entire articulation. Manual `sleep()`
  is rejected; this adapter keeps bodies awake.
- Kinematic bodies use a free collision body welded to a separate non-colliding
  mocap target. Contacts receive the body's simulated linear and angular velocity;
  rendered poses follow that collision body. Tracking is compliant, so bodies can
  lag or yield under load rather than behaving as infinitely stiff platforms.
  Their authored mass/inertia controls the response; colliderless anchors default
  to mass 1 and diagonal inertia [1, 1, 1]. Gravity is compensated. Teleport moves
  both body and target; model rebuilds preserve the outstanding target.
- Static and dynamic friction must be equal. Restitution maps to MuJoCo's contact
  damping ratio, so impact behavior differs from Rapier.
- Scalar limits require a nonzero interval. Use fixed joints or locked generic
  degrees of freedom for a locked coordinate. An infinite distance maximum
  requires a zero minimum.
- MuJoCo collides mesh convex hulls, so a `trimesh` collider collides as the convex
  parts the core decomposes when the world is built, or as one hull if it was added
  or edited later. Complete regular height grids with planar cells on static bodies
  use native heightfields with a 1 mm base. Explicit `convexHull` colliders always
  use one hull.

The adapter validates heap-view types at the WASM boundary and frees owned native
objects explicitly. MuJoCo simulation warnings surface as errors instead of
silently accepting a solver reset.

Run `pnpm --filter @drawcall/physics-mujoco test` for Node contract tests. The car,
ragdoll, and scale packages additionally test their full scenes against MuJoCo.

To see decomposition in action, run `pnpm --filter @drawcall/example-scale dev`
and open **Convex vs decomposed** (`?demo=decomposition&backend=mujoco`).
Two identical frame meshes use different collision approximations: the left
cube rests on a single hull spanning the opening; the right cube falls through
an opening preserved by CoACD. Both frames are authored before `buildWorld()`.
**Replay drops** resets the world and reuses the prepared colliders.
