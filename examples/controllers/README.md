# Controller playground

Four playable game controllers built on `@drawcall/physics`. Choose Character, Car, Plane, or Quadcopter, then switch between Rapier and MuJoCo. The playground uses the controller source bundled with the [game-controller skill](../../packages/physics/skills/physics-controllers/SKILL.md) and the core `CapsuleMotor`.

```sh
pnpm install
pnpm --filter @drawcall/physics build
pnpm --filter @drawcall/physics-rapier build
pnpm --filter @drawcall/physics-mujoco build
pnpm --filter @drawcall/example-controllers dev
```

| Mode | Controls | Try |
| --- | --- | --- |
| Character | WASD/arrows move, Space jump, Shift run, C crouch | Stairs, slopes, low tunnel, moving platforms |
| Car | W/S drive/reverse, A/D steer, Space brake, Shift handbrake | Slalom, drifting, jump ramp, bumps, pushable crates |
| Plane | W/S climb/dive, A/D bank, Shift boost, Space slow | Fly through the visual guide rings and release to level out |
| Quadcopter | WASD/arrows move, Space/C climb/descend, Q/E yaw, Shift boost | Hover, change direction, land on the gold pad |

R resets the scene. Drag to orbit; scroll to zoom. Touch buttons mirror the keyboard. Changing mode or backend restarts the page. Direct links accept `?controller=car&backend=mujoco` (controller values: `character`, `car`, `plane`, `quadcopter`).

Use the bundled skill to copy and adapt a controller for your own game. These are game-oriented movement recipes; vehicle handling is deliberately assisted.

```sh
pnpm --filter @drawcall/example-controllers test
pnpm --filter @drawcall/example-controllers typecheck
pnpm --filter @drawcall/example-controllers build
```
