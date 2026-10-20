---
name: physics
description: Build and simulate physics in Three.js scenes with @drawcall/physics — rigid and soft bodies, cloth, ropes, colliders, joints, drives, triggers, raycasts, and worlds built by a backend package such as @drawcall/physics-rapier or @drawcall/physics-mujoco. Use when adding physics to a three.js scene, authoring or cloning physics objects, driving joints, reacting to contacts or overlaps, or writing a new simulation backend.
---

# @drawcall/physics

Bodies, attachments, colliders, joints, and triggers are ordinary `Object3D`s. You build, clone, import, and export them without any engine; a backend package turns a scene into a running world. Reach for:

- `@drawcall/physics` for the scene objects, `JointDrive`, `clone`, and shared types.
- `@drawcall/physics-rapier` or `@drawcall/physics-mujoco` for `buildWorld`. Both simulate the same scene objects. Rapier suits games and interactive scenes that change often; MuJoCo suits robotics and articulated mechanisms, needs joints authored as a tree, and recompiles its model whenever the scene changes. Each backend package documents what its engine handles differently.
- `@drawcall/physics-usd` to import or export the objects as USD Physics.
- `@drawcall/physics/backend` only when writing a backend; scene code never needs it.

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
world.update(delta); // each frame, elapsed seconds
world.dispose(); // when done
```

## The scene graph is the membership

A world simulates whatever is under its `scene` and nothing else. Before every update, step, query, and command it re-reads the scene: objects added under it join, objects removed leave. There is no `world.add`; use `scene.add` and `removeFromParent`. `object.world` tells whether an object has joined.

Consequences that are easy to get wrong:

- A scene has one live world, and an object belongs to one world. Building a second world for the same scene throws until the first is disposed; an object under two worlds' scenes throws.
- A joint whose bodies are not under the same scene throws on the next update.
- Joining captures scale and joint anchors. Finish placing and scaling an object before it joins; to change them later, replace the object.
- A failed join is not isolated: until the broken object is fixed or removed, updates, queries, and commands on not-yet-joined objects all throw. A failed `buildWorld` disposes the world and leaves the scene untouched.

Commands on objects (`body.setVelocity`, `teleport`, `applyImpulse`, `getContacts`, `trigger.overlaps`, `joint.getState`, …) go to the world whose scene holds the object and join it on the spot, so a command right after `scene.add` works in any loop order, including inside step callbacks. Outside any built world's scene, or while the world is still building, they throw. The equivalent world methods take the object as first argument.

A body joins with its current pose and its `velocity` option, which is what `reset()` restores, so give a body that must replay its launch speed through `velocity` rather than `setVelocity`. Reset touches only the simulation: objects added mid-run stay in the scene, and step callbacks, drive targets, and application state keep their values, so a replay that spawns objects must remove them and reset its own state too.

Per-step control logic, such as spawning, grabbing, or setting kinematic targets, goes in `onBeforeStep`, which runs at the fixed rate regardless of the frame rate; `update` itself belongs only in the frame loop.

## Reading results

Read poses from `body.matrixWorld`; steps and `teleport` keep it current. It includes scale, so use `splitTransform(matrix).pose` when you need the rigid part. To move a joined body, use `teleport` (the dynamic bodies jointed to it move along) or, for a kinematic body, `setKinematicTarget` every frame.

## Colliders

Without explicit colliders, a body generates one per visual mesh according to its `colliders` option, so a body made of meshes collides out of the box. Adding any explicit collider under a body turns generation off for that body; then the colliders' transforms place the shapes and the meshes are purely visual. Use explicit colliders when the visual mesh is too detailed or the wrong shape, and `colliders: false` for a body that should not collide at all (it then needs complete mass properties if dynamic).

Materials and collision groups set on a collider replace the body's. `body.getColliders()` and `body.getMaterial(collider)` show what will actually be simulated.

Scale rules that reject at join time: boxes and meshes accept positive nonuniform scale; spheres and capsules need uniform scale, cylinders equal X/Z scale; moving bodies need uniform ancestor scale. Zero, negative, or sheared transforms, nested bodies, nested triggers, partial draw ranges, and automatic colliders on instanced, skinned, or morphed meshes are rejected.

Geometry edits are noticed the way three.js renderers notice them: replace the position or index attribute, or set `needsUpdate = true` after an in-place edit.

### Concave meshes

A `"trimesh"` collider stays exact where the engine supports it (static bodies in Rapier). Elsewhere (moving bodies in Rapier, everything in MuJoCo) the world decomposes it into convex parts with CoACD:

- `buildWorld` waits for the decomposition of meshes present at build time.
- A mesh added or edited later collides as its convex hull while decomposing in the background, then switches to its parts (recomputing mass unless `mass` is explicit). `await world.decompose(root)` waits until the parts are in, so the next update collides them.
- Only a mesh's latest geometry is decomposed: a mesh edited every frame stays on its hull and `decompose` does not resolve while the edits continue.
- CoACD needs closed, consistently wound surfaces. An open surface fails the update that builds it, and keeps failing every update until the mesh is edited or removed. Use `"convexHull"` when one hull is enough.

In browser bundles decomposition runs in a module worker. With Vite before 8, add `optimizeDeps: { exclude: ["@drawcall/physics"] }`, or the worker cannot load and decomposition fails with an error saying so. Elsewhere (Node, Bun, Deno, unbundled browsers) it blocks the calling thread; Node 20.16+ caches parts in `node_modules/.cache/@drawcall/physics`, which you can delete to decompose afresh.

## Soft bodies

Use `SoftBody` for deformable solids, cloth, and ropes. Choose the elasticity law with the backend: the backend's `buildWorld` doc comment lists the supported combinations. Unsupported combinations throw; switching engines does not silently substitute a different material model. Material values need calibration against the physical sample and mesh resolution; a label such as TPU does not determine them.

The constructor fixes rest shape and topology. `setMaterial` replaces physical properties while preserving deformation and velocity. A soft body owns its deformed render geometry; connect the visible mesh to that geometry rather than the original rest mesh:

```ts
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

An attachment selects simulation nodes in rest-local coordinates and retains their initial offsets from its rigid target, or from the world with `target: null`. A region box usually survives mesh-resolution changes better than node indices. Keep attached nodes clear of target colliders: attachments do not disable their contact. Disable and re-enable an attachment to release and restore its original anchors.

Place and uniformly scale the body before it joins. Read deformed world nodes with `getPositions`; `matrixWorld` remains the body's frame, not its deformation. Apply forces or impulses to the body to distribute a total load, or supply a world point to load the nearest node. Replace the body to change topology. USD export currently rejects soft bodies and attachments.

For physical comparisons, choose a constitutive model supported by the backend and use measured geometry, density, stiffness, and losses. Spring-frequency profiles depend on mesh resolution; they are not substitutes for a measured Young's modulus. Check mesh and timestep convergence against a known load or motion before trusting a real-world prediction. Backend-specific elastic laws and damping conventions are not interchangeable calibrations.

Material damping dissipates deformation; it does not necessarily dissipate the motion of the whole assembly. Use `setLinearDamping` on soft or rigid bodies for an explicitly calibrated linear motion-loss model, independently of material damping. It does not model air flow or suspension friction automatically. Match a measured decay or drag law rather than interpreting a visually pleasing settling time as validation.

## Joints

A joint connects `body0` (the base, or `null` for the world) to `body1`. By default its own world placement defines both anchors, so position and rotate the joint object where the hinge, slider, or socket is, and add it to the scene (anywhere under it, often under one of its bodies). Pass `frame0` and `frame1` instead when the anchors are known in body-local coordinates, as when the bodies are not yet in place.

Make `body0` the side closer to the root and author a tree: reduced-coordinate backends such as MuJoCo build their kinematic tree from it.

`GenericJoint` with per-axis `dofs` covers any combination the specific joint types do not, including a fully free joint used only to carry drives.

## Drives

A `JointDrive` attached with `setDrive` acts on one joint coordinate: a position target with stiffness makes a servo (a constant one a spring), a velocity target with damping a motor; generic joints take one drive per axis. `model: "acceleration"` lets one set of gains behave alike on light and heavy bodies.

To drag a dynamic body while contacts still stop it, do not teleport it; hold it with a kinematic hand through a free joint whose translation drives pull it to the hand:

```ts
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

## Events and triggers

- Body `contactbegin`/`contactend` fire on both bodies of a pair; `Trigger` `enter`/`exit` fire per rigid body overlapping the colliders under the trigger. A trigger placed under a body follows it and ignores it. Triggers do not detect other triggers.
- Events dispatch after each step, before `onAfterStep`. Removing an object ends its contacts and overlaps with events; `reset()` and `dispose()` clear them silently.
- A throwing listener does not stop delivery; the errors are rethrown after every queued event is delivered.

## Cloning

Clone anything that holds joints or soft attachments with `clone(root)` from this package; it reconnects constraints to the cloned bodies. Cloning those constraints directly throws. Rigid clones share geometry and materials; soft clones own separate deformed geometry and start from their rest shape. Clones are simulated once added under a world's scene.

## Conventions

SI units and radians throughout. Configuration takes tuples; runtime state uses three.js vectors, quaternions, and matrices. Features only some engines support, such as torsional and rolling friction or drive `frictionLoss` and `armature` (MuJoCo only), are rejected by backends that cannot honor them rather than approximated, so an error at join time usually means "this backend cannot do that", not a bug in your scene.

## Writing a backend

A backend extends `PhysicsWorld` from `@drawcall/physics`, implements its protected hooks, and exports a `buildWorld` that awaits the engine and returns `build(new MyWorld(...))` from `@drawcall/physics/backend`. The base class owns membership, the fixed clock, callbacks, events, and validation, so hooks receive only joined members and validated arguments; the hooks' doc comments and the helpers exported from `@drawcall/physics/backend` describe the rest. The Rapier package's test suite in the repository is the executable contract: start a new backend from them, and keep each engine limitation as a test next to its rejection.
