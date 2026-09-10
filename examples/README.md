# Physics examples

Two independent Vite packages, sharing only `view.ts`, CSS, and TypeScript defaults.

```sh
pnpm install
pnpm build
pnpm --filter @drawcall/example-ragdoll dev
pnpm --filter @drawcall/example-car dev
```

Run either dev command and open its Vite URL. Each package also has its own
`build` and `typecheck` scripts.

- **Ragdoll:** a falling articulated body using spherical and limited revolute joints.
- **Car:** four powered wheels, spring/damper suspension, front steering, fixed rear
  knuckles, and a bump course. Starts with an automatic run; W/S drives, A/D steers,
  Space brakes, and T restarts the automatic test.

Both support P to pause, R to reset, and mouse orbit/zoom. Each package has a small
`main.ts` and its physics model. Car settings live in `car/model.ts`; its controller
and road are separate files. The car uses rigid tires and a simplified electric
drivetrain at 120 Hz with 32 solver iterations.

`pnpm --filter @drawcall/example-car test` runs bump-course, braking, reverse,
rear-alignment, and stationary-steering regressions in `car/test/`.
