# Physics examples

Three independent Vite packages, sharing `view.ts`, backend selection, CSS, and TypeScript defaults.

```sh
pnpm install
pnpm build
pnpm --filter @drawcall/example-ragdoll dev
pnpm --filter @drawcall/example-car dev
pnpm --filter @drawcall/example-scale dev
```

Run a dev command and open its Vite URL. Each package also has its own
`build` and `typecheck` scripts. Select **Rapier** or **MuJoCo WASM** in the
backend dropdown. Changing it reloads the example and starts a fresh simulation;
the selection is preserved in the URL (`?backend=mujoco`). Browser builds emit
the official WASM asset locally. The headless tests also exercise MuJoCo in Node.js.

- **Ragdoll:** a falling articulated body using spherical and limited revolute joints.
  Drag any limb with mouse or touch: `@pmndrs/pointer-events` captures the pointer, and
  a kinematic hand pulls the limb through a force-limited generic-joint spring, so the
  body stays dynamic and the floor still stops it.
- **Car:** four powered wheels, spring/damper suspension, front steering, fixed rear
  knuckles, and a bump course. Starts with an automatic run; W/S drives, A/D steers,
  Space brakes, and T restarts the automatic test.

**Collider scale checks:** 115 selectable cases cover every collider, automatic and
explicit construction, uniform/nonuniform scale on bodies, ancestors, colliders,
and combined transforms. Checks compare visual/collider bounds, measure resting
contact, and repeat after removal/recreation in the same running world. Static
shapes support falling test cubes. Kinematic lifts move vertically using kinematic
targets and carry their cubes. The falling compound is one dynamic body with three
automatic convex colliders; triangle meshes remain static-only. Direct buttons
open the falling compound and moving lift. Expected failures and stretched
sphere/capsule mesh alternatives are included. Expand the results list to inspect
each outcome; N selects the next case, X removes it, R recreates it, and P pauses.

`pnpm --filter @drawcall/example-scale test` runs the same cases headlessly.

Ragdoll and car support P to pause, R to reset, and mouse orbit/zoom; orbiting pauses while a limb is held. Each package has a small
`main.ts` and its physics model. Car settings live in `car/model.ts`; its controller
and road are separate files. The car uses rigid tires and a simplified electric
drivetrain at 120 Hz with 32 solver iterations.

`pnpm --filter @drawcall/example-car test` runs bump-course, braking, reverse,
rear-alignment, and stationary-steering regressions in `car/test/`.
