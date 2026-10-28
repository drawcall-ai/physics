# @drawcall/physics-usd

USD Physics import and export for [`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme) scene objects. No simulation backend is needed to exchange scenes.

```sh
npm install three @drawcall/physics @drawcall/physics-usd
```

```ts
import { PhysicsUSDExporter, PhysicsUSDLoader } from "@drawcall/physics-usd";

const bytes = await new PhysicsUSDExporter().parseAsync(scene);
const { scene: imported, gravity } = await new PhysicsUSDLoader().parseAsync(
  bytes,
);
```

See the shipped [physics-usd skill](skills/physics-usd/SKILL.md) for importing scenes into simulations and choosing an export state. Types and doc comments describe the supported USD subset.
