---
name: physics-usd
description: Exchange @drawcall/physics scenes through USD Physics with @drawcall/physics-usd. Use when importing a USD scene into a simulation, exporting authored or simulated state, or duplicating an imported assembly.
---

# @drawcall/physics-usd

Use this package to exchange visual and rigid-physics scene objects without starting a simulation. Use the [physics skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics/skills/physics/SKILL.md) for authoring and running those objects. The loader and exporter doc comments describe the supported format and features; conversion rejects unsupported physics instead of silently dropping it.

## Bring a stage into a world

The imported stage supplies both scene membership and gravity, so pass it directly to a backend when creating a new world:

```ts
import { PhysicsUSDLoader } from "@drawcall/physics-usd";
import { buildWorld } from "@drawcall/physics-rapier";

const stage = await new PhysicsUSDLoader().loadAsync(url);
const world = await buildWorld({ ...stage, fixedDelta: 1 / 120 });
```

To bring an assembly into an existing world, add the imported scene beneath that world's scene. Choose deliberately whether to retain the existing world's gravity; adding objects does not apply the imported stage's gravity. Duplicate imported assemblies with `clone` from `@drawcall/physics` so their joints reconnect to the cloned bodies.

## Choose the export state

Decide whether the file should capture the current simulation or the body's authored join state. Export the current scene for a snapshot; call `world.reset()` first for the join state. Reset does not undo application-level additions or removals, so restore those separately when exporting a reproducible starting scene.

```ts
import { PhysicsUSDExporter } from "@drawcall/physics-usd";

const bytes = await new PhysicsUSDExporter().parseAsync(scene, {
  gravity: [0, -9.81, 0], // choose the gravity of the exported stage
});
```

For exchange with other USD consumers, check their support for the authored joints and drives before relying on equivalent simulation. In particular, distance drives are this package's extension. Prepare unsupported composition and coordinate conventions in the authoring tool before import; do not discard physical settings merely to make a file pass validation.
