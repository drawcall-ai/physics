---
name: physics-controllers
description: Build and tune game character, car, plane, and quadcopter controls with @drawcall/physics. Use the core capsule motor and bundled controller source for playable movement on Rapier or MuJoCo; not high-fidelity vehicle simulation.
---

# Game controllers

Use `CapsuleMotor` from `@drawcall/physics` for capsule collision movement. For game-specific acceleration, jumping, handling, or flight assistance, copy the matching TypeScript files below into your application and adapt them. These files ship with the installed physics package; they are source to own, not package import paths or additional runtime exports. See the [physics skill](../physics/SKILL.md) for scene and backend setup.

## Choose the movement

| Movement | Source to copy | Use it for |
| --- | --- | --- |
| Character | [character/controller.ts](assets/character/controller.ts) | Walking, jumping, crouching, and moving platforms over the core capsule motor |
| Car | [car/controller.ts](assets/car/controller.ts), [car/model.ts](assets/car/model.ts) | Tunable arcade driving with suspension, grip, braking, and handbrake slides |
| Plane | [plane/controller.ts](assets/plane/controller.ts), [plane/model.ts](assets/plane/model.ts), [flight.ts](assets/flight.ts) | Assisted cruise, climbing, banked turns, and leveling |
| Quadcopter | [quadcopter/controller.ts](assets/quadcopter/controller.ts), [quadcopter/model.ts](assets/quadcopter/model.ts), [flight.ts](assets/flight.ts) | Responsive movement, hovering, climbing, and yaw |

Preserve the selected directory's layout. Either aircraft also needs `flight.ts` beside its directory. The `.js` relative imports refer to the corresponding TypeScript modules when compiled. The vehicle models include simple meshes so the copied code runs without external assets; replace those visuals while retaining the bodies, wheel mounts, and mass properties the controller uses. Character visuals belong under an ordinary `Object3D` whose origin is at the feet.

## Connect input and simulation

Create the model and add it to the scene before building the world, then create its controller. Translate keyboard, gamepad, or AI intent into the controller's `input`; keep that translation outside the movement code. Controllers already register their physics-step callback, so the render loop only needs to advance the world.

After copying the car directory to your app's `controllers/car`:

```ts
import { Scene } from "three";
import { buildWorld } from "@drawcall/physics-rapier";
import { createCar } from "./controllers/car/model.js";
import { CarController } from "./controllers/car/controller.js";

const scene = new Scene();
const car = createCar();
scene.add(car.body); // Add your ground and obstacles before building the world.
const world = await buildWorld({ scene, fixedDelta: 1 / 120 });
const controller = new CarController(world, car);
controller.input.throttle = 1;
// In the frame loop: world.update(elapsedSeconds).
```

Keep the page or game responsible for scene reset: reset the world first, then clear the controller's state. Dispose the controller before discarding it so its step callback is removed. Animate propellers and other purely visual effects in the render layer.

## Adapt the character motor

Start with the copied character controller when you need jumping, crouching, buffered input, or platform transport. It uses the core `CapsuleMotor` for geometry; keep game policy in your copy so collision fixes can arrive through the physics dependency.

For a different locomotion policy, construct `CapsuleMotor` directly and combine its movement operations in your own fixed-step callback. Recover shallow overlap, request displacement and ground snapping as appropriate, then commit the resulting pose to the visual object. Movement is in world space with an upright Y-axis capsule. The motor does not apply gravity or decide when a character may jump. Follow completed support poses when carrying a character on moving platforms; the bundled character controller runs after physics for this reason.

The motor's object is visual, not a native rigid body. Use game-level interaction logic if a character must push objects or trigger interactions; capsule queries alone do not generate native contact events. Check the motor's declaration for geometry constraints and query limitations before changing dimensions, terrain, or spawn placement.

## Tune and check the result

The vehicle controllers intentionally use game assists. Adjust the source's tuning values and movement response for your game instead of adding physical subsystems that do not improve its controls. Keep their forces and torque on the physics clock so contact response remains active. The recipes assume Y-up, unit scene scale, and vehicle noses along local −Z.

After adapting a recipe, check the intended behavior on your chosen backend: character walls, steps and ceiling clearance; car turning, stopping and airborne wheels; aircraft release-to-level or release-to-hover and collision recovery. Check reset and disposal as well. If supporting both backends, compare those outcomes within useful tolerances rather than expecting identical trajectories.
