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
import { setupWorld } from "@drawcall/physics-mujoco";
import { RigidBody, BoxCollider } from "@drawcall/physics";

const world = await setupWorld({ fixedDelta: 1 / 120 });
const box = new RigidBody({ mass: 1 });
box.position.y = 2;
box.add(new BoxCollider());
world.update(world.fixedDelta);
console.log(box.position.y);
world.dispose();
```

With Vite, import the WASM as an asset and pass its URL. Add `@mujoco/mujoco`
as a direct dependency when importing this asset from application code:

```ts
import { setupWorld } from "@drawcall/physics-mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";

const world = await setupWorld({ wasmUrl, solverIterations: 50 });
```

Other bundlers must serve `mujoco.wasm` and supply its URL through `wasmUrl`.
The examples demonstrate both development and production asset loading.
`setupWorld` installs the new default world and shares the loaded WASM module;
each world owns and frees its own model and simulation data.

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
- Kinematic targets use MuJoCo mocap bodies. Their prescribed poses participate in
  contacts; MuJoCo does not infer mocap velocities from successive targets.
- Static and dynamic friction must be equal. Restitution maps to MuJoCo's contact
  damping ratio, so impact behavior differs from Rapier.
- Scalar limits require a nonzero interval. Use fixed joints or locked generic
  degrees of freedom for a locked coordinate. An infinite distance maximum
  requires a zero minimum.
- MuJoCo collides mesh convex hulls. Convex meshes use native mesh assets. Regular,
  complete height grids with planar cells become native heightfields. Other static
  triangle meshes become individual triangular prisms extending 1 mm behind each
  face; concavities remain, but seams and very thin features can affect contacts.
  Heightfields have a 1 mm base. Neither representation is a general exact,
  zero-thickness triangle-mesh collider.

The adapter validates heap-view types at the WASM boundary and frees owned native
objects explicitly. MuJoCo simulation warnings surface as errors instead of
silently accepting a solver reset.

Run `pnpm --filter @drawcall/physics-mujoco test` for Node contract tests. The car,
ragdoll, and scale packages additionally test their full scenes against MuJoCo.
