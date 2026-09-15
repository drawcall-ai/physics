# Physics API decisions

## Shape dimensions are configuration

Primitive dimensions (`size`, `radius`, `length`, `height`) are immutable
constructor options. They describe the collider being created. Changing them
requires replacing that collider. A previous writable field does not establish
that runtime mutation is needed.

Material, sensor, and collision-group settings use methods. Three.js transforms
and supported mesh geometry/child edits remain separate scene-authoring operations;
they do not require dimension setters on primitive colliders.

## Mass belongs to the backend

Core accepts immutable `mass`, `centerOfMass`, `diagonalInertia`, and
`principalAxes`. The backend derives defaults from colliders during preparation. A total mass
override preserves collider mass ratios; a complete mass/COM/inertia override
replaces collider contributions. Other partial combinations are rejected. Core does not calculate convex hulls, volume centroids, or inertia.

This follows the established engine boundary: [Rapier computes mass properties
from attached colliders](https://rapier.rs/docs/user_guides/javascript/rigid_body_mass_properties/),
[PhysX provides shape-based mass helpers](https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/_api_build/classPxRigidBodyExt.html),
and [MuJoCo infers inertial data during model compilation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler).
Explicit authoritative properties must not receive an additional collider mass
contribution. Principal axes default to identity for a complete specification.

## Prepare physical data without stepping

A body's linear velocity is measured at its center of mass. For a rotating body,
velocity at a joint anchor depends on the anchor's offset from that center.
Prepared slider measurements use Rapier's
[`velocityAtPoint`](https://rapier.rs/javascript3d/classes/RigidBody.html#velocityAtPoint),
so the backend owns both inference and the point-velocity conversion.

`world.update(0)` prepares completed construction without advancing simulation.
Normal updates also prepare it. Queries never initialize or freeze an unfinished
assembly. If a rotating slider needs collider-inferred COM before preparation,
`getState()` throws a preparation error. Body velocity reads, nonrotating slider
reads, and joint reads with sufficient explicit mass data remain available during
construction. AuthoringWorld can use explicit COM; it cannot infer backend data.

This deliberately excludes exact collider-derived velocity before preparation.
The alternative would duplicate the engine's mass calculations or create hidden
backend objects during a read. Explicit preparation also has an established
analogue in [MuJoCo's `mj_forward`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-forward),
which computes derived state without integrating time.

## One joint concept, one effort command

A joint restricts relative body motion; “constraint” describes that role and does
not need a second application-level object or manager. The world owns one joint
record containing its backend binding and pending effort.

`joint.setEffort(value)` replaces that pending value. The fixed-step loop resolves
it into native body forces or torques immediately before the solver, then clears
it after the completed step. Deferred consumption supports replacement, cancellation, and independent body
forces on prepared joints. Unprepared effort commands fail; existing assemblies
are prepared before the first before-step callback. Applying
native forces on every setter call would need extra bookkeeping to undo previous
commands.

Effort lifetime is explicit, not universal: [Rapier forces accumulate and persist](https://rapier.rs/docs/user_guides/javascript/rigid_body_forces_and_impulses/),
and [MuJoCo leaves its control/force inputs unchanged](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#user-inputs).
This package keeps its specified one-fixed-substep lifetime. Controllers submit
another command in `onBeforeStep` when continued effort is desired. There is no
public apply/flush phase for callers and no separate effort manager.

## Native motors, fixed mechanical limits

The [superseding issue decision](https://github.com/drawcall-ai/physics/issues/6#issuecomment-5677664067)
keeps ordinary position/velocity actuation in physics through `JointMotor`.
[Rapier supplies native position/velocity motors](https://rapier.rs/javascript3d/classes/UnitImpulseJoint.html); the car does not duplicate their solver with a
JavaScript PD loop. Binding, gains, model, and actuator ceiling are immutable;
targets and enabled state are mutable. The motor starts untargeted and inactive.
Nonzero direct effort requires disabling an active motor; there is no automatic
control-mode switching. Physics owns actuation; robotics owns command timing and
controller restrictions.

A separate motor follows an [established actuator model](https://mujoco.readthedocs.io/en/3.3.5/computation/#actuation-model), while immutable gains
and mechanical joint limits are deliberate scope choices. Engine APIs commonly
allow live changes; this integration does not require exposing them. Physical
limits stay in joint constructor options. Body type switching remains available
through `setType`, preserving an existing capability.

## Raycast policy

Closest-hit queries normalize directions so distances are in meters, exclude
sensors unless requested, and return the exit surface for an inside origin. These
are explicit package policies, not industry requirements. The current filtering
and geometry tests cover them; changing them would add migration without reducing
the implementation meaningfully.
