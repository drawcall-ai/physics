# @drawcall/physics

Physics scene objects for Three.js. Build rigid and soft bodies, colliders, and joints as ordinary `Object3D`s, then simulate them with a backend.

| Package | Purpose |
| --- | --- |
| `@drawcall/physics` | Scene objects and shared contracts |
| [`@drawcall/physics-rapier`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-rapier) | Rapier simulation |
| [`@drawcall/physics-mujoco`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-mujoco) | MuJoCo simulation in browsers and Node |
| [`@drawcall/physics-fem`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-fem) | Cached reduced FEM components |
| [`@drawcall/physics-usd`](https://github.com/drawcall-ai/physics/tree/main/packages/physics-usd) | USD Physics import and export |

## Quick start

```sh
npm install three @drawcall/physics @drawcall/physics-rapier
```

```ts
import { BoxGeometry, Mesh, Scene } from "three";
import { RigidBody } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-rapier";

const scene = new Scene();
const body = new RigidBody({ mass: 1 });
body.add(new Mesh(new BoxGeometry()));
scene.add(body);

const world = await buildWorld({ scene });
world.update(1 / 60); // each frame, with elapsed seconds
world.dispose(); // when done
```

For MuJoCo, import `buildWorld` from `@drawcall/physics-mujoco` instead. The shipped [physics skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics/skills/physics/SKILL.md) covers scene authoring and links to each package's workflows.

## Examples

In a clone of the [repository](https://github.com/drawcall-ai/physics):

```sh
pnpm --filter @drawcall/example-ragdoll dev  # articulated ragdoll, pointer grab
pnpm --filter @drawcall/example-car dev      # powered car with suspension
pnpm --filter @drawcall/example-controllers dev # character, car, plane, quadcopter
pnpm --filter @drawcall/example-scale dev    # scaled colliders and decomposition
pnpm --filter @drawcall/example-softbody dev # grip a block, cloth, or cord with reduced FEM pads
```

The [controller playground](examples/controllers) shows four game controllers built on the shared API: capsule movement, a raycast car, an assisted plane, and a hovering quadcopter. Switch modes and backends in the page; every controller runs on Rapier and MuJoCo.
