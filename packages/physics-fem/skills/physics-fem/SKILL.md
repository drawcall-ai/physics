---
name: physics-fem
description: Prepare cached reduced FEM soft bodies from static Three.js solids or GLTS assets, attach them to mounts, and simulate them with @drawcall/physics-mujoco in Node or browsers.
---

# Reduced FEM components

Use `FEMLoader` for a deformable solid whose rest shape, material, and mounting area are known before simulation. Use ordinary `SoftBody` shapes for cloth, ropes, or another backend's native soft-body model; see the [physics skill](https://github.com/drawcall-ai/physics/blob/main/packages/physics/skills/physics/SKILL.md).

## Prepare and mount a component

Start with a closed, consistently wound solid in metres. Finish scaling, local mesh transforms, and source-asset loading before preparation. Place the prepared component in the scene before building the world. Keep its geometry and visual hierarchy fixed while it is simulated; rebuild the component when its rest shape or physical recipe changes.

Choose a mounting region that represents the real fixture and a contact region covering every surface another object may touch. Keep other colliders clear of uncovered surfaces, including near the mounting boundary. An unsupported intersection is a modeling error, not an invitation to disable the contact guard.

```ts
import { Scene } from "three";
import { SoftAttachment } from "@drawcall/physics";
import { FEMLoader } from "@drawcall/physics-fem";
import { buildWorld } from "@drawcall/physics-mujoco";

const loader = new FEMLoader();
const finger = await loader.loadAsync({
  url: "/finger.glts",
  // Illustrative values and regions; replace with the component's measurements.
  material: {
    density: 1200,
    elasticity: { model: "neoHookean", youngModulus: 5e6, poissonRatio: 0.45 },
  },
  mount: { min: [-0.02, -0.001, -0.02], max: [0.02, 0.005, 0.02] },
  contact: { min: [-0.02, 0.03, -0.02], max: [0.02, 0.06, 0.02] },
});
if (!finger.model) throw new Error("Expected a reduced FEM component");
const scene = new Scene();
scene.add(
  finger,
  new SoftAttachment({
    body: finger,
    region: finger.model.mount,
    target: null,
  }),
);
const world = await buildWorld({ scene });
world.update(1 / 60); // each rendered frame, with elapsed seconds

// When the component is no longer needed:
world.dispose();
finger.removeFromParent();
loader.dispose();
```

Use exactly one enabled attachment covering the baked mount. Plan the component around the mount and motion supported by the [MuJoCo backend](https://github.com/drawcall-ai/physics/blob/main/packages/physics-mujoco/skills/physics-mujoco/SKILL.md); a prepared component does not gain new deformation or attachment capabilities when placed in a larger scene.

## Prepare an existing mesh

Pass a completed rest mesh as `object` instead of a `url`:

```ts
import { FEMLoader } from "@drawcall/physics-fem";

const finger = await new FEMLoader().loadAsync({
  object: mesh,
  material,
  mount,
  contact,
});
scene.add(finger);
```

Add the returned body to display the deforming visual, rather than also adding the original rest mesh. This also works inside a GLTS script; share `@drawcall/physics` and `@drawcall/physics-fem` with scripts so the host's world recognizes the body; see the [GLTS skill](https://github.com/drawcall-ai/glts/blob/main/packages/glts/skills/glts/SKILL.md).

## Prepare for deployment

The same request can prepare a cache miss in Node or the browser without a Python installation. Prepare common components before deployment and include their cache files so visitors can load them immediately. Prepared files are keyed by the snapshotted rest geometry and physical request, not by source files, so keep both identical between preparation and loading, and coordinate the preparation directory with the deployed cache URL through `setCachePath`.

Allow workers and WebAssembly in the application's content security policy when visitors may need to prepare uncached components.

## Validate the intended task

Start with a supported material law and measured geometry, density, stiffness, and losses. A linear model is useful for small strains and rotations; more general material laws do not by themselves guarantee an adequate reduced deformation space or real-time performance.

Compare load versus deflection, contact shape, and force against a refined model or measurements. Check mesh and timestep convergence across the intended loading range. Refine the volume mesh and the contact representation separately when identifying the source of an error; a visually accurate surface is not a certificate of mechanical accuracy.
