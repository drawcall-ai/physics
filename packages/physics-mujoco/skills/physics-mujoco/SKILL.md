---
name: physics-mujoco
description: Author and control @drawcall/physics scenes with @drawcall/physics-mujoco. Use for articulated mechanisms, MuJoCo contact tuning, or combining reduced FEM components with rigid bodies; use the core physics skill for shared scene authoring.
---

# @drawcall/physics-mujoco

Use the [physics skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics/skills/physics/SKILL.md) for scene membership, colliders, joints, and the simulation loop. This backend suits mechanisms whose articulated structure stays stable while their drives move them. Its `buildWorld`, `MujocoWorld`, and `MujocoWorldOptions` doc comments describe supported models, constraints, and loading requirements.

## Assemble before controlling

Lay out a mechanism from its base toward its moving links, with each joint's `body0` on the base side. Finish placing the bodies and joint frames before building the world. An articulated link moves through its joint drives; use kinematic targets to move a kinematic base. Direct body velocity and teleport commands are constrained by the articulation, so they are not substitutes for joint control.

Attach drives during authoring, retain them, and update their targets from `onBeforeStep`. Keep additions, removals, collider edits, and drive replacement outside the control loop when possible: structural edits cost substantially more than target changes.

```ts
import { JointDrive } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";

// hinge is an authored RevoluteJoint under scene.
const drive = new JointDrive({ stiffness: 100, damping: 10, maxForce: 40 });
hinge.setDrive(drive);
const world = await buildWorld({ scene });
world.onBeforeStep(() => {
  drive.setTarget({ position: desiredAngle });
});
// In the frame loop: world.update(deltaSeconds).
```

## Tune the assembled interaction

When a grasp creeps, distinguish contact slip from motion at a driven joint. Contact behavior involves both collider materials and the world's friction settings; joint holding involves the drive. Tune against the assembled mechanism and its load. Kinematic targets and soft attachments can lag under load, so measure the simulated pose or deformation when evaluating tracking. Stiff, thin deformable meshes may need smaller timesteps and more solver iterations.

For a reduced deformable component, use the [physics-fem skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics-fem/skills/physics-fem/SKILL.md) to prepare its model, then combine the resulting soft body and attachment with the rigid scene. Check this backend's reduced-model constraints before baking: the mount motion and contact region must fit the intended interaction. Exercise the expected load and contact range before treating the component as interchangeable with its full mesh.
