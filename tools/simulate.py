# /// script
# requires-python = "==3.12.*"
# dependencies = [
#   "newton[importers]==1.6.0",
#   "usd-core==26.3",
#   "PyOpenGL>=3.1,<4",
#   "pyglet>=2.1.6,<3",
#   "imgui-bundle>=1.92,<2",
#   "imageio==2.37.4",
#   "imageio-ffmpeg==0.6.0",
# ]
# ///
"""Open, record, or check a USDZ using Newton's importer and XPBD solver."""

import argparse
import math
from contextlib import ExitStack
from pathlib import Path

import newton
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--video", type=Path, help="Record an MP4 instead of opening a window")
    mode.add_argument("--check", action="store_true", help="Simulate without graphics (for CI)")
    parser.add_argument("--seconds", type=float, default=5, help="Duration for recording/checking")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")

    builder = newton.ModelBuilder()
    stage = Usd.Stage.Open(str(args.asset.resolve()))
    imported = builder.add_usd(
        stage,
        apply_up_axis_from_stage=True,
    )
    # XPBD supports standalone joint graphs. Only skip Newton's articulation-membership
    # requirement; keep its shape, inertia, and structural validation enabled.
    model = builder.finalize(device="cpu", skip_validation_joints=True)
    if not model.body_count:
        raise ValueError("No enabled rigid bodies imported")
    solver = newton.solvers.SolverXPBD(model, iterations=10)
    state, next_state = model.state(), model.state()
    initial = state.body_q.numpy().copy()
    static = [
        imported["path_body_map"][str(prim.GetPath())]
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        and not UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        and str(prim.GetPath()) in imported["path_body_map"]
    ]
    control = model.control()
    collision = newton.CollisionPipeline(model)
    contacts = collision.contacts()
    fps, substeps = 30, 32
    frames = math.ceil(args.seconds * fps)

    with ExitStack() as resources:
        viewer = writer = None
        if not args.check:
            from newton.viewer import ViewerGL

            viewer = ViewerGL(width=960, height=720, headless=bool(args.video), vsync=True)
            resources.callback(viewer.close)
            viewer.set_model(model)
            bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
            center = np.array(bounds.GetMidpoint())
            radius = max(float(np.linalg.norm(bounds.GetSize())) / 2, 1)
            viewer.set_camera(center + np.array([radius, radius * 0.7, radius * 2]), -18, -115)
        if args.video:
            import imageio.v2 as imageio

            writer = resources.enter_context(imageio.get_writer(str(args.video), fps=fps))

        frame = 0
        while frame < frames if args.check or args.video else viewer.is_running():
            if viewer is None or not viewer.is_paused():
                for _ in range(substeps):
                    state.clear_forces()
                    if viewer:
                        viewer.apply_forces(state)
                    collision.collide(state, contacts)
                    solver.step(state, next_state, control, contacts, 1 / (fps * substeps))
                    state, next_state = next_state, state
                if not np.isfinite(state.body_q.numpy()).all():
                    raise RuntimeError(f"Nonfinite body transform at frame {frame}")
                frame += 1
            if viewer:
                viewer.begin_frame(frame / fps)
                viewer.log_state(state)
                viewer.end_frame()
                if writer:
                    writer.append_data(viewer.get_frame().numpy())

    if static and not np.allclose(state.body_q.numpy()[static], initial[static], atol=1e-5):
        message = "Compatibility failure: Newton moved an authored disabled/static rigid body"
        if args.check:
            raise RuntimeError(message)
        print(message)
    displacement = np.linalg.norm(state.body_q.numpy()[:, :3] - initial[:, :3], axis=1).max()
    print(f"Newton: {model.body_count} bodies, {model.joint_count} joints, {frame} frames; "
          f"maximum displacement {displacement:.3f} m")


if __name__ == "__main__":
    main()
