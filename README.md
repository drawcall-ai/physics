# Drawcall Physics

Three.js physics authoring, simulation, and USD interchange in separate packages.

| Package                    | Purpose                                                          |
| -------------------------- | ---------------------------------------------------------------- |
| `@drawcall/physics`        | Scene objects, validation, automatic colliders, assembly cloning |
| `@drawcall/physics-rapier` | Optional Rapier simulation                                       |
| `@drawcall/physics-usd`    | USD Physics import and export                                    |

```sh
pnpm install
pnpm check
```

## Examples

Run `pnpm --filter @drawcall/example-ragdoll dev` or
`pnpm --filter @drawcall/example-car dev`, then open the Vite URL.
The examples demonstrate an articulated ragdoll and a powered car with suspension
on a bump course. See [examples](examples) for controls and source.

## Authoring contracts

- Positions use meters, masses kilograms, angles radians, and time seconds.
- `setupWorld()` initializes the chosen backend and installs the default world before asset execution.
- `RigidBody` extends `Group`. Colliders and joints extend `Object3D`.
- Bodies and joints register automatically. New bodies can be constructed at any time;
  their backend resources are prepared before the next simulation step.
- Bodies and joints keep their constructor settings in `object.options`. Body references
  and world ownership are fixed; joint drive and limit settings remain mutable.
- A body captures its world at construction, while a joint inherits its bodies’ world.
  An explicit body `{ world }` overrides the default. Cross-world joints fail.
- `dispose()` releases a body or joint; removing visuals alone does not end its lifetime.
- Authoring without simulation uses `setDefaultWorld(new AuthoringWorld())` from
  `@drawcall/physics`.
- `BoxCollider`, `SphereCollider`, `CapsuleCollider`, `CylinderCollider`, and
  `MeshCollider` supply explicit shapes. Explicit colliders take precedence over automatic generation. Colliders must belong to a body, including static ones.
- Capsules and cylinders use local Y. Capsule `length` is the straight section.
- `FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `SphericalJoint`, and
  `DistanceJoint` connect bodies. `body0: null` connects to the world.
- Joint world placement defines both local anchors at preparation. Alternatively,
  supply both `frame0` and `frame1` as body-local `Matrix4` transforms.
- Revolute and prismatic joints default to local Y. Their initial prepared pose
  defines zero. Distance joints support minimum and maximum separation in the
  authoring model; runtime backends validate their supported subset.
- `body.getColliders()` returns collider objects, `body.getMaterial(collider)`
  resolves their physics materials, and `joint.getFrame(index, matrix)` writes
  a body-local anchor. Bodies and colliders expose their Three.js `matrixWorld`.
- V1 rejects scaled/sheared physics transforms, nested bodies, and automatic
  colliders on skinned, instanced, or morph-deformed meshes. Triangle meshes
  require static bodies. Dynamic bodies require positive mass or density.
- Explicit collider `collisionGroups: { membership, filter }` uses two unsigned
  16-bit bitmasks. An interaction requires both membership/filter intersections.
  Adapter support is validated; USD export currently rejects this extension.
- Bodies and joints support native `.clone()` and `.copy()`, which preserve joint
  body references. Use `clone(root)` from `@drawcall/physics` to clone an ordinary
  hierarchy and remap internal joint references, like Three.js SkeletonUtils.
  Geometry and materials remain shared; clones register in the source world.

Adapters report unsupported features instead of discarding them. See each package
for its simulation and interchange limitations. Geometry and visual material
ownership remains with the application.

## Third-party verification

Use [tools/simulate.py](tools/README.md) to open exported USDZ in Newton, record
an MP4, or run a headless CPU check. This uses an independent importer and solver.
