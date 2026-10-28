---
name: physics-rapier
description: Run @drawcall/physics scenes with @drawcall/physics-rapier and control driven joints. Use when choosing Rapier for interactive scene changes, implementing continuous hinge motion, or adapting a scene from another backend.
---

# @drawcall/physics-rapier

Use the [physics skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics/skills/physics/SKILL.md) for scene authoring and the fixed-step loop. Rapier suits interactive scenes whose bodies, colliders, and joints change during play. Read the `buildWorld` and `RapierWorld` doc comments when selecting materials or bringing a scene from another backend; supported elastic laws and drive features differ.

## Control a hinge across turns

A multi-turn position goal needs intermediate targets based on the joint's current continuous angle. Set the next target in `onBeforeStep`, keeping it less than half a turn ahead; a fixed jump to a distant angle is rejected. A velocity drive is another option when the task is continuous rotation rather than arrival at a position.

```ts
import { JointDrive } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-rapier";

// hinge is an authored RevoluteJoint under scene.
const drive = new JointDrive({ stiffness: 100, damping: 10, maxForce: 40 });
hinge.setDrive(drive);
const world = await buildWorld({ scene });
world.onBeforeStep(() => {
  const current = hinge.getState().position;
  const remaining = desiredAngle - current;
  const advance = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, remaining));
  drive.setTarget({ position: current + advance });
});
// In the frame loop: world.update(deltaSeconds).
```

Keep feedback drives and effort-only actuation distinct when a total force cap matters. Combining them cannot share one cap in this backend. For physical comparisons across backends, recalibrate the supported material and drive model against the same motion or load instead of copying gains and assuming equivalent results.
