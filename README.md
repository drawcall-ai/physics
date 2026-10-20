# @drawcall/physics

Physics scene objects for Three.js. Rigid and soft bodies, colliders, and joints are ordinary
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

## Usage

The package ships an agent skill in
[`skills/`](https://github.com/drawcall-ai/physics/tree/main/packages/physics/skills):
`physics` covers worlds, bodies, cloth, ropes, deformable solids, joints, triggers, and
choosing a backend. Each backend
package's README lists its engine-specific behavior.

## Examples

In a clone of the [repository](https://github.com/drawcall-ai/physics):

```sh
pnpm --filter @drawcall/example-ragdoll dev # articulated ragdoll, pointer grab
pnpm --filter @drawcall/example-car dev     # powered car with suspension
pnpm --filter @drawcall/example-scale dev   # scaled colliders and decomposition
pnpm --filter @drawcall/example-hammock dev # cloth, ropes, and deformable cargo
```

## Development

```sh
pnpm install
pnpm --filter @drawcall/physics build:coacd # builds the CoACD WASM; needs CMake and Python 3
pnpm check
```
