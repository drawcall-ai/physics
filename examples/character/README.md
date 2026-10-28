# Character playground

A playable, shared capsule controller over `@drawcall/physics`, with a Rapier/MuJoCo switch. The course includes stairs, walkable and steep ramps, a low tunnel, walls, an elevator, and a rotating platform.

```sh
pnpm install
pnpm --filter @drawcall/physics build
pnpm --filter @drawcall/physics-rapier build
pnpm --filter @drawcall/physics-mujoco build
pnpm --filter @drawcall/example-character dev
```

WASD/arrows move relative to the camera, Space jumps, Shift runs, C crouches, and R resets. Drag to orbit and scroll to zoom. Touch devices have movement buttons. Changing the backend restarts the scene; `?backend=mujoco` selects MuJoCo directly.

`controller.ts` handles acceleration, jumping, buffering, coyote time, and platform momentum. `motor.ts` resolves capsule motion through the shared world's `castCapsule` and `capsuleContacts` queries. The character owns its pose and follows completed simulation steps, including the actual translation and rotation of its support. It has no native rigid body, so the same movement code runs on both engines. `model.ts` builds the physical course independently of rendering.

This is an example to develop the controller, not a finished general character API. It supports upright capsules with downward Y gravity and a 0.32 m step limit. It does not push dynamic bodies or participate in native contacts/triggers. Capsule queries currently scan rigid colliders; soft bodies are excluded, and MuJoCo heightfields are unsupported. Sweep queries see obstacle poses at one simulation instant, so fast incoming obstacles and crushing need further work. Rapier concave-mesh edge cases also need broader coverage before using arbitrary terrain.

Spawn and reset the capsule clear of geometry. Recovery handles shallow overlap; deep symmetric overlaps can produce unusable native contacts even against cylinders or capsules, and queries throw when that failure is detected. Rapier can also miss a sweep from exact touching between aligned capsules; the playground's sweep clearance reduces reliance on exact contact.

```sh
pnpm --filter @drawcall/example-character test
pnpm --filter @drawcall/example-character typecheck
pnpm --filter @drawcall/example-character build
```

The same behavioral tests run on both backends: thin walls, sliding, jumping, step limits, slopes, ceiling clearance, platform transport, and support removal. Backend suites test query geometry, filtering, initial overlap, and live scene changes separately.
