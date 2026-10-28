---
name: physics
description: Build and simulate Three.js scenes with @drawcall/physics, including rigid and soft bodies, colliders, joints, drives, triggers, cloning, and custom simulation backends.
---

# Physics scenes

Author bodies, colliders, constraints, and triggers as ordinary Three.js objects, then choose a backend to simulate the scene. Use the [Rapier skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics-rapier/skills/physics-rapier/SKILL.md) for interactive scenes and the [MuJoCo skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics-mujoco/skills/physics-mujoco/SKILL.md) for articulated mechanisms and reduced FEM. Check the backend's supported material and joint combinations before choosing an authoring approach; changing backends is not a material calibration.

Use [physics-fem](https://github.com/drawcall-ai/physics/blob/main/packages/physics-fem/skills/physics-fem/SKILL.md) to prepare mounted deformable solids, and [physics-usd](https://github.com/drawcall-ai/physics/blob/main/packages/physics-usd/skills/physics-usd/SKILL.md) to exchange authored scenes through USD Physics.

## Build the scene before the world

Work in metres, kilograms, seconds, and radians. Finish object scale, joint placement, and mounting regions before adding an assembly to a running scene. Let the scene graph control world membership instead of maintaining a second object list.

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
hinge.position.x = 0.06; // the joint's world placement is the hinge
scene.add(frame, door, hinge);

const world = await buildWorld({ scene });
world.update(1 / 60); // each frame, with elapsed seconds
world.dispose(); // when done
```

Use `world.update` in the frame loop and put continuous forces, kinematic targets, and drive control in `onBeforeStep` so they follow the physics clock. Use forces or drives for movement that must respond to contact; reserve teleporting for deliberate repositioning.

A reset workflow must also restore application state: remove spawned objects and reset controller targets alongside `world.reset`. Fix or remove an invalid scene member before retrying a failed update.

## Choose collision geometry

Visual meshes can supply automatic colliders for a first prototype. Switch to explicit colliders when collision geometry should be simpler or different from the visible shape; once doing so, describe the whole body's collision shape explicitly.

Use a convex hull when one enclosing shape is sufficient. For a concave component, use a closed, consistently wound triangle mesh and let the backend prepare convex parts where required. If newly loaded or edited geometry must have its final collision shape before interaction, await `world.decompose(root)` before resuming the simulation. Finish mesh edits before waiting for that preparation.

## Connect soft-body mechanics and visuals

Choose a material law supported by the backend and keep the undeformed geometry separate from the mesh used for display. For example, this Rapier cloth uses the body's deforming geometry and a world attachment:

```ts
import { Mesh, PlaneGeometry } from "three";
import { SoftAttachment, SoftBody } from "@drawcall/physics";

const cloth = new SoftBody({
  shape: {
    kind: "surface",
    geometry: new PlaneGeometry(1, 1, 12, 12),
    thickness: 0.001,
  },
  material: {
    density: 300,
    elasticity: {
      model: "spring",
      stretch: { frequency: 30, dampingRatio: 1 },
    },
  },
});
cloth.add(new Mesh(cloth.geometry, visualMaterial));
const pin = new SoftAttachment({
  body: cloth,
  region: { min: [-1, 0.49, -0.01], max: [1, 0.51, 0.01] },
  target: null,
});
scene.add(cloth, pin);
```

Use rest-local region bounds for attachments that should survive changes in mesh resolution. Place the body and its target together before joining the scene, and check that the mount is clear of colliders that could fight the attachment.

For physical comparisons, use measured geometry, density, stiffness, and losses. Check mesh and timestep convergence against a known load or motion; spring-frequency profiles need recalibration after remeshing. Calibrate motion damping separately from material losses when bulk swing or drift must decay. A visually pleasing settling time alone does not validate a material.

## Assemble joints and drives

Place joints at the intended hinge, slider, or socket in the assembled scene. Use explicit local frames when importing anchors from an asset whose parts are not yet placed. For articulated robots, make the base-to-child direction consistent throughout the mechanism and check the backend's topology restrictions.

Choose position control for a servo or spring and velocity control for a motor. Set drive targets from your controller each physics step. To grab a dynamic body while keeping contact response, connect it to a kinematic hand with translation drives instead of repeatedly teleporting it.

Given a dynamic `body` and a world-space grab `point`:

```ts
import { Matrix4 } from "three";
import { GenericJoint, JointDrive, RigidBody } from "@drawcall/physics";

const hand = new RigidBody({ bodyType: "kinematic", colliders: false });
hand.position.copy(point); // world grab point
const free = "free" as const;
const hold = new GenericJoint({
  body0: hand,
  body1: body,
  frame0: new Matrix4(),
  frame1: new Matrix4().makeTranslation(body.worldToLocal(point.clone())),
  dofs: {
    transX: free,
    transY: free,
    transZ: free,
    rotX: free,
    rotY: free,
    rotZ: free,
  },
});
for (const axis of ["transX", "transY", "transZ"] as const)
  hold.setDrive(
    axis,
    new JointDrive({
      model: "acceleration",
      stiffness: 1000,
      damping: 63,
      maxForce: 600,
    }).setTarget({ position: 0 }),
  );
hand.add(hold);
scene.add(hand);
// each step: hand.setKinematicTarget(pose); to release: hand.removeFromParent()
```

## React and reuse

Use body contact events for impact responses and triggers for interaction zones. Keep event handlers focused on application changes; perform whole-simulation restart or teardown from the application's lifecycle code.

Clone a constrained assembly with `clone(root)` before placing it in the destination scene. Keep the connected bodies under the cloned root when they should be duplicated together, then apply the new assembly's placement before it joins its world.

## Add a backend

For a custom engine integration, extend `PhysicsWorld`, implement its documented protected hooks, and expose a factory that awaits engine initialization before returning `build(new MyWorld(options))` from `@drawcall/physics/backend`. Use that backend entry point for engine integration; ordinary scene authoring only needs the package's main exports.
