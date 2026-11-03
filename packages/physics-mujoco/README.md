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
