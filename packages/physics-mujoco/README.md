# @drawcall/physics-mujoco

MuJoCo backend for
[`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme),
built on the official [`@mujoco/mujoco`](https://github.com/google-deepmind/mujoco/tree/main/wasm)
WASM bindings. It runs single-threaded in Node.js and in browsers without cross-origin
isolation. The core guide covers the shared contracts; this page lists only
MuJoCo-specific behavior.

```sh
npm install three @drawcall/physics @drawcall/physics-mujoco
```

```ts
import { buildWorld } from "@drawcall/physics-mujoco";

const world = await buildWorld({ scene, fixedDelta: 1 / 120 });
```

Node.js loads the packaged WASM automatically. In browsers, serve `mujoco.wasm` and pass
its URL. With Vite, add `@mujoco/mujoco` as a direct dependency and import the asset:

```ts
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";

const world = await buildWorld({ scene, wasmUrl });
```

`@mujoco/mujoco` imports Node's `module` builtin; webpack browser builds need
`resolve: { fallback: { module: false } }`.

## Options

`MujocoWorldOptions` extend the shared options:

- `solverIterations` defaults to 50.
- `frictionCone`: `"pyramidal"` (default) is more robust for kinematic contact at small
  steps; `"elliptic"` models friction faithfully.
- `frictionImpedanceRatio` is MuJoCo's `impratio` (default 1). At 1 a resting or
  grasped object creeps; grasping needs around 50. Values above 1 require elliptic cones.
- `wasmUrl`: see above.

## Model rebuilds

MuJoCo simulates one compiled model. Objects joining or leaving, edits to colliders or
their geometry, and enabling, disabling, or re-driving joints recompile it. A rebuild
keeps the members' poses, joint velocities, and kinematic targets, but costs far more than
an edit in Rapier. Drive target changes need no rebuild. Scale is fixed once compiled.

Every `trimesh` collides as convex parts; those that join or change after the build need
`await world.decompose(object)` before they join. Exception: a complete regular height
grid with planar cells on a static body becomes a native heightfield.

## Engine differences

- Constrained joints must form a tree: one parent joint per `body1`, which must be
  dynamic, and no cycles. Distance joints and fully free generic joints may connect any
  bodies. Locked coordinates must be aligned when a joint is created or re-enabled.
- Bodies held by a `FixedJoint` share one MuJoCo body, so `collideConnected` throws.
- Scalar limits need a nonzero range; an infinite distance maximum needs a zero minimum.
- `setVelocity` needs a free root body; move articulated bodies with drives.
  Teleporting a dynamic body moves its whole articulation and throws when that
  articulation hangs from the world or a static or kinematic base.
- `sleep()` throws; bodies stay awake.
- Kinematic bodies are free bodies welded to a mocap target: contacts see their velocity,
  but they track compliantly and can lag under load. Colliderless kinematic bodies
  default to mass 1 and unit inertia.
- Static and dynamic friction must be equal; friction combines as the larger value.
  Restitution maps to the contact damping ratio, so impacts differ from Rapier.
- Hinge and slider drives are native actuators. Distance drives and drives on fully free
  generic joints are implicit springs. `maxVelocity` becomes joint damping.
- Collision groups map to `contype`/`conaffinity`; needing more than 32 bits throws.
- MuJoCo warnings and non-finite state throw instead of silently resetting.
