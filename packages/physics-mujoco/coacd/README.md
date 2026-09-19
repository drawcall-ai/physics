# CoACD WASM

Pinned CoACD 1.0.11 (b678aa0802996fa03e1ec0e68bd05acf8cd20cf9), compiled with
Emscripten 5.0.2. `bind.cpp` is our small typed-array adapter to the upstream
algorithm. `build.sh` reproduces the ES module and WASM for web, workers, and Node.
No native compiler is required by consumers.

The build disables third-party preprocessing; inputs must be closed manifold
meshes. The TypeScript adapter welds identical render vertices and validates edges
before passing the mesh to CoACD. Native validation and decomposition errors fail
the build. Decomposition currently runs on the caller's thread.

Upstream source, including third-party components, is included in `source.tar.gz`
for inspection and rebuilding. CoACD is MIT licensed (see LICENSE). It includes
CDT under MPL-2.0, Bullet convex hull code under its permissive license, Antti
Kuukka's public-domain QuickHull, and John Burkardt's LGPL Sobol routines.
See `licenses/` and the notices in the source archive. CDT is unmodified, pinned
at ec03b309fd18102ab1da069f2edf3b37be5d1fb3 by CoACD's submodule.

Upstream: https://github.com/SarahWeiii/CoACD/tree/1.0.11
CDT source: https://github.com/artem-ogre/CDT/tree/ec03b309fd18102ab1da069f2edf3b37be5d1fb3
