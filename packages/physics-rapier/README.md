# @drawcall/physics-rapier

Rapier simulation backend for [`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme), for rigid and deformable bodies in interactive Three.js scenes.

```sh
npm install three @drawcall/physics @drawcall/physics-rapier
```

```ts
import { buildWorld } from "@drawcall/physics-rapier";

const world = await buildWorld({ scene });
world.update(deltaSeconds);
```

See the shipped [physics-rapier skill](skills/physics-rapier/SKILL.md) for control workflows. Types and doc comments describe supported features and backend constraints.
