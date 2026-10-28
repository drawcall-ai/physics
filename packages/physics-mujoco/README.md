# @drawcall/physics-mujoco

MuJoCo simulation backend for [`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme), for articulated mechanisms and deformable bodies in Three.js scenes. Runs in Node.js and browsers with embedded WASM.

```sh
npm install three @drawcall/physics @drawcall/physics-mujoco
```

```ts
import { buildWorld } from "@drawcall/physics-mujoco";

const world = await buildWorld({ scene, fixedDelta: 1 / 120 });
world.update(deltaSeconds);
```

See the shipped [physics-mujoco skill](skills/physics-mujoco/SKILL.md) for authoring and control workflows. Types and doc comments describe supported features and backend constraints.

`world.capsuleContacts` and `world.castCapsule` query rigid primitives and convex mesh parts. Casts use conservative advancement against the current obstacle poses, with a fixed capsule orientation; they do not predict moving obstacles. Heightfields are rejected unless excluded from the query, and soft bodies and triggers are omitted. Queries temporarily reuse a disabled native capsule and restore its state without stepping the simulation. Candidate geometry is currently scanned linearly.

Deep symmetric overlaps can defeat the native distance query, including a capsule centered inside a cylinder. Queries throw when the native result cannot resolve a contact normal; general recovery from deep penetration is unsupported.
