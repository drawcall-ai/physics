# Independent USD Physics simulation

This runs the third-party [Newton](https://github.com/newton-physics/newton)
USD importer, XPBD solver, and native viewer. It does not use our USD importer,
Three.js, or Rapier. The input USDZ is not converted or modified.

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then run from
this repo. uv selects Python 3.12 and installs the script's dependencies in an isolated environment.

```sh
# Open Newton's interactive viewer
uv run tools/simulate.py ragdoll.usdz

# Record five seconds to MP4
uv run tools/simulate.py ragdoll.usdz --video ragdoll.mp4 --seconds 5

# CPU simulation without a display, suitable for CI
uv run tools/simulate.py ragdoll.usdz --check --seconds 5
```

The script preserves authored gravity, uses CPU simulation, and preserves the file exactly as Newton imports it. It reports imported body/joint counts and rejects nonfinite
transforms. Compare counts and inspect motion; a successful run alone does not
prove every exported property is supported or numerically equivalent.

Newton 1.6 normally requires joints to belong to articulations. Our files use
standalone USD Physics joints, supported by XPBD. The script disables only
articulation-membership validation, leaving structural and physical-data checks
on. It does not add articulation schemas or rewrite the joint graph.

CPU simulation supports macOS, Linux, and Windows. Viewing/recording needs an
OpenGL-capable environment; `--check` needs no display. Initial runs compile Warp
kernels and may take longer. Videos show Newton's rendering of the imported scene.

The browser scenes in `examples/` are local physics examples, not independent
interoperability checks.

## Static-body interoperability

Static groups now export collision geometry without `PhysicsRigidBodyAPI`.
Newton imports the floor as static collision geometry, and the ragdoll lands on it.
The older disabled-rigid-body representation was valid USD but imported as dynamic
in Newton 1.6. The launcher still detects that failure in older files.

The XPBD configuration uses 10 iterations and 32 substeps per recorded frame
(960 Hz) to stabilize the ragdoll with connected-body collisions enabled. These
are solver settings, not modifications to the USDZ or its collision flags.
