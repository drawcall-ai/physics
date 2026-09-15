# Physics API decisions

## Mass belongs to the backend

Core accepts immutable `mass`, `centerOfMass`, `diagonalInertia`, and
`principalAxes`. The backend infers unspecified properties from colliders during
preparation. Core does not calculate convex hulls, volume centroids, or inertia.

This follows the established engine boundary: [Rapier computes mass properties
from attached colliders](https://rapier.rs/docs/user_guides/javascript/rigid_body_mass_properties/),
[PhysX provides shape-based mass helpers](https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/_api_build/classPxRigidBodyExt.html),
and [MuJoCo infers inertial data during model compilation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler).
Explicit authoritative properties must not receive an additional collider mass
contribution. Partial overrides retain engine-derived values for omitted fields.

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
it after the completed step. Deferring consumption supports construction-time
commands, cancellation, final joint frames, and independent body forces. Applying
native forces on every setter call would need extra bookkeeping to undo previous
commands.

Effort lifetime is explicit, not universal: [Rapier forces accumulate and persist](https://rapier.rs/docs/user_guides/javascript/rigid_body_forces_and_impulses/),
and [MuJoCo leaves its control/force inputs unchanged](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#user-inputs).
This package keeps its specified one-fixed-substep lifetime. Controllers submit
another command in `onBeforeStep` when continued effort is desired. There is no
public apply/flush phase for callers and no separate effort manager.
