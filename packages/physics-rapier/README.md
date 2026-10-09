# @drawcall/physics-rapier

[Rapier](https://rapier.rs) backend for
[`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme).
The guide there covers worlds, objects, commands, and events; this page lists only
Rapier-specific behavior.

```sh
npm install three @drawcall/physics @drawcall/physics-rapier
```

```ts
import { buildWorld } from "@drawcall/physics-rapier";

const world = await buildWorld({ scene, gravity: [0, -9.81, 0] });
world.update(deltaSeconds);
```

`buildWorld` takes the shared world options. Omitting `solverIterations` keeps
Rapier's default.

## Bodies and colliders

- Body type and `canSleep` are fixed when the body joins; damping and gravity scale
  update live.
- Body and collider scale are captured on joining. A later scale edit throws;
  recreate the object. New colliders capture their own scale.
- Triangle meshes stay exact on static bodies. On moving bodies they collide as convex
  parts: those added or edited after the build need `await world.decompose(object)`
  before they join.
- Static and dynamic friction must be equal.
- An explicit mass without explicit inertia is split across colliders by volume.
- A collider rebuild that fails keeps the previous colliders.
- Teleporting an assembly jointed to the world or a static or kinematic base applies
  the pose and lets the solver correct it on the next step. Joints the teleported
  assembly straddles restart their turn count.

## Joints and drives

- Joints with `body0: null` attach to one fixed ground body, which lives as long as
  the world.
- Anchors and limits are captured on joining. Disabling a joint removes its Rapier
  joint and keeps the anchors.
- Distance joints are Rapier spring joints: the minimum must be zero, a finite maximum
  becomes the rope limit, and a drive acts along the spring axis.
- Drive gains map to Rapier's force- or acceleration-based motors with `maxForce` as
  the motor limit. A drive without gains configures no motor.
- Effort is applied as a force pair for one step, clamped to `maxForce` on its own. A
  capped drive that combines gains with effort is rejected, as is `maxVelocity`.
- Rapier's motor takes the shortest arc, so a revolute position target with stiffness
  must stay within π of the current angle; longer moves need intermediate targets.

## Triggers and raycasts

- Triggers are Rapier sensors. A trigger not under a moving body rides a private
  kinematic carrier, because Rapier skips fixed/fixed pairs.
- Raycasts test colliders directly, so bodies that joined or were teleported since the last step
  are hit without a step.
