# Controller playground

Four playable game controllers built on `@drawcall/physics`. Choose Character, Car, Plane, or Quadcopter, then switch between Rapier and MuJoCo. The same controller code runs on both engines.

```sh
pnpm install
pnpm --filter @drawcall/physics build
pnpm --filter @drawcall/physics-rapier build
pnpm --filter @drawcall/physics-mujoco build
pnpm --filter @drawcall/example-controllers dev
```

## Play

| Mode | Controls | Try |
| --- | --- | --- |
| Character | WASD/arrows move, Space jump, Shift run, C crouch | Stairs, slopes, low tunnel, moving platforms |
| Car | W/S drive/reverse, A/D steer, Space brake, Shift handbrake | Slalom, drifting, jump ramp, bumps, pushable crates |
| Plane | W/S climb/dive, A/D bank, Shift boost, Space slow | Fly through the rings and release the controls to level out |
| Quadcopter | WASD/arrows move, Space/C climb/descend, Q/E yaw, Shift boost | Hover, change direction, land on the gold pad |

R resets the current scene. Drag to orbit; scroll to zoom. Touch buttons mirror the keyboard. Changing mode or backend restarts the page. Direct links accept `?controller=car&backend=mujoco` (controller values: `character`, `car`, `plane`, `quadcopter`).

## Build a controller

Each mode has a `model.ts` and `controller.ts` (the character's environment is `character/course.ts`). The model creates ordinary Three.js meshes and physics objects. The controller accepts intent through `input`, reads the shared physics state, and applies motion each fixed step. There are no native engine imports in the controllers.

```ts
const car = createCar();
scene.add(car.body);
const world = await buildWorld({ scene, fixedDelta: 1 / 120 });
const controller = new CarController(world, car);
controller.input.throttle = 1;
// Your render loop calls world.update(elapsedSeconds).
// When removing this controller:
controller.dispose();
```

`main.ts` owns the world, rendering, input and scene reset. `vehicles.ts` and `character/demo.ts` bind the page to the individual controllers; `demo.ts` describes only that page lifecycle. Controller `reset()` clears its own state; the page calls `world.reset()` separately. This keeps the reusable movement code independent of keyboard controls, the camera and the course.

- **Character:** `character/motor.ts` handles capsule sweeps, sliding, stairs and shallow penetration recovery. `character/controller.ts` adds acceleration, crouching, jumping, coyote time, buffered input and platform carry. It runs after physics to use the platforms' completed poses.
- **Car:** one dynamic chassis and four suspension rays. Springs support the body; bounded longitudinal/lateral forces provide acceleration and grip. Speed-sensitive steering and applying tire forces near the center of mass keep cornering approachable. The handbrake reduces rear grip. The wheels are visual children. Tune the named values in `car/model.ts`.
- **Plane:** forward-speed assistance, lateral air grip, speed-dependent gravity support and damped bank/pitch targets produce arcade flight. Release the controls to level out; bank to turn. This is deliberately assisted game movement, not an aerodynamic simulation.
- **Quadcopter:** desired velocity plus gravity compensation provides responsive movement and hovering. Releasing input brakes to rest; acceleration tilts the physical body. Yaw and attitude correction use bounded torque. Propellers are visual, without rotor dynamics.

The vehicles apply forces in `onBeforeStep`. `applyForce(force, point?)` and `applyTorque(torque)` accumulate for the next step only. Their vectors and points are world-space. `getVelocityAtPoint(point)` includes rotation around the actual center of mass, so the car can measure wheel motion relative to moving ground. `flight.ts` is the small damped attitude assist shared by both aircraft. Contacts still come from the backend; these controllers never teleport a body to steer it.

## Why these techniques

The [Just Cause 4 vehicle-physics talk](https://gdcvault.com/play/1026035/Vehicle-Physics-and-Tire-Dynamics) argues for designer-friendly tire models with fewer parameters for arcade handling. [Rapier's vehicle controller](https://rapier.rs/docs/user_guides/javascript/vehicle_controller/) uses a single chassis with raycast wheels; [PhysX vehicle documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Vehicles.html) describes suspension loads and friction-limited tire forces. This example uses that small, tunable foundation, without a drivetrain or detailed tire simulation. Native vehicle controllers are coupled to their engines, so a short implementation over shared queries and forces is a better fit here.

## Scope and checks

This is a game-controller teaching example. Wheel rays have no volume, so sharp curbs need a different wheel query or authored ramp. The flight rings are visual guides; posts, ground and vehicle bodies collide. Aircraft assistance is intentional and does not model stalls, propwash or rotor thrust allocation. Controllers assume Y-up; the character additionally requires downward Y gravity. Vehicle bodies use explicit mass/inertia and unscaled scene-root placement.

The character uses an upright virtual capsule, with a 0.32 m step limit. It does not push bodies or join native contact/trigger events. Capsule queries scan rigid colliders; soft bodies are excluded and MuJoCo heightfields are unsupported. Sweeps see a snapshot of obstacle poses, so crushing and fast incoming obstacles need further work. Spawn clear of geometry: recovery handles shallow overlap; unusable deep native contacts throw. Rapier can miss sweeps from exact touching between aligned capsules. Broader concave-terrain coverage is needed before using arbitrary terrain.

```sh
pnpm --filter @drawcall/example-controllers test
pnpm --filter @drawcall/example-controllers typecheck
pnpm --filter @drawcall/example-controllers build
```

Behavioral tests run on both backends: character movement and support; car suspension, steering, braking, reversing and handbrake grip; aircraft cruise, bank, climb, hover, collision response and timestep convergence. Shared API tests separately cover torque lifetime and point velocity. The tests check gameplay behavior within tolerances rather than identical engine trajectories.
