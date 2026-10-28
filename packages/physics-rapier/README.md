# @drawcall/physics-rapier

Rapier simulation backend for [`@drawcall/physics`](https://github.com/drawcall-ai/physics#readme), for rigid and deformable bodies in interactive Three.js scenes.

```sh
npm install three @drawcall/physics @drawcall/physics-rapier
```

```ts
import { buildWorld } from "@drawcall/physics-rapier";

const world = await buildWorld({ scene });
world.update(deltaSeconds);
```

See the shipped [physics-rapier skill](skills/physics-rapier/SKILL.md) for control workflows. Types and doc comments describe supported features and backend constraints.

`capsuleContacts` and `castCapsule` query rigid colliders at arbitrary poses, excluding triggers and soft bodies. Casts translate the capsule against the current obstacle poses. Queries currently scan collider candidates, and contacts report one native witness per collider. Use positive clearance and penetration recovery for character movement; initial overlap and exact tangent contact against concave triangle meshes retain native numerical limitations.

Deep, nearly concentric overlaps can also defeat native contact queries on primitives, including cylinders and capsules. Queries throw when a zero-distance contact reports a witness away from the obstacle surface. This guard detects that native failure; it does not provide general recovery from deep penetration.

Exact touching can also produce an unreliable native normal or missed sweep on convex primitives, including aligned capsules. A valid surface witness does not rule out this limitation; start casts with positive clearance.
