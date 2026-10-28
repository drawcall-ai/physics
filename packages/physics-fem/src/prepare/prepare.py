"""Build a reduced component with the shared JavaScript/WASM geometry services."""

import geometry
import numpy as np
import trimesh
from pyodide.ffi import to_js
from skfem import MeshTet

from contact import contact_surface, interpolation, inside, overlaps
from reduce import discretize, reduce
from surface import boundary, visual_basis


async def prepare(request):
    positions = np.asarray(request["positions"], dtype=float).reshape(-1, 3)
    indices = np.asarray(request["triangles"])
    if not np.isfinite(indices).all() or np.any(indices != np.floor(indices)):
        raise ValueError("FEM triangle indices must be integers")
    triangles = indices.astype(np.int64).reshape(-1, 3)
    if len(positions) < 4 or not np.isfinite(positions).all() or np.any(triangles < 0) or np.any(triangles >= len(positions)):
        raise ValueError("Invalid FEM surface positions or indices")
    surface = trimesh.Trimesh(vertices=positions, faces=triangles, process=False)
    if not surface.is_watertight or not surface.is_winding_consistent or surface.volume <= 0:
        raise ValueError("FEM source must be a closed, consistently outward-wound volume")
    material = request["material"]
    if material["elasticity"].get("dampingRatio", 0):
        raise ValueError("FEM reduction uses dampingTime, not per-element dampingRatio")
    options = request.get("mesh") or {}
    cell_size = options.get("cellSize", float(np.ptp(positions, axis=0).max() / 8))
    order, max_dofs = options.get("order", 2), options.get("maxDofs", 60)
    if order not in [1, 2] or not isinstance(max_dofs, int) or max_dofs < 9:
        raise ValueError("FEM mesh order must be 1 or 2 and maxDofs an integer ≥9")
    if not np.isfinite(cell_size) or cell_size <= 0:
        raise ValueError("FEM cellSize must be positive and finite")
    volume = await geometry.tetrahedralize(
        to_js(positions.ravel()), to_js(triangles.astype(np.uint32).ravel()), cell_size
    )
    points = np.asarray(volume.positions.to_py()).reshape(-1, 3)
    tetrahedra = np.asarray(volume.tetrahedra.to_py()).reshape(-1, 4)
    mesh = MeshTet(points.T, tetrahedra.T)
    scalar = discretize(mesh, order)
    nodes = scalar.doflocs.T
    fixed_nodes = np.flatnonzero(inside(nodes, request["mount"]))
    if len(fixed_nodes) < 3 or np.linalg.matrix_rank(nodes[fixed_nodes] - nodes[fixed_nodes].mean(axis=0)) < 2:
        raise ValueError("FEM mount must select at least three non-collinear nodes")
    faces, face_cells = boundary(mesh, scalar)
    fixed_mask = inside(nodes, request["mount"])
    contacts = faces[overlaps(nodes, faces, request["contact"]) & ~fixed_mask[faces].all(axis=1)]
    if not len(contacts):
        raise ValueError("FEM contact region does not overlap an unconstrained surface triangle")
    source_contact = overlaps(positions, triangles, request["contact"])
    source_mount = inside(positions, request["mount"])[triangles].all(axis=1)
    control, coarse_triangles = contact_surface(
        nodes, contacts, positions, triangles[source_contact & ~source_mount], max_dofs, request["mount"]
    )
    control_fixed = np.flatnonzero(inside(control, request["mount"]))
    control_free = np.setdiff1d(np.arange(len(control)), control_fixed)
    boundary_nodes = np.setdiff1d(np.unique(contacts), fixed_nodes)
    if not len(boundary_nodes) or not len(control_free):
        raise ValueError("FEM contact region has no unconstrained nodes")
    weights, contact_distance = interpolation(nodes[boundary_nodes], control, coarse_triangles)
    if contact_distance > cell_size / 2:
        raise ValueError("FEM contact skin deviates by more than half cellSize; increase maxDofs")
    prescribed = np.kron(weights[:, control_free], np.eye(3))
    retained = (boundary_nodes[:, None] * 3 + np.arange(3)).ravel()
    fixed = (fixed_nodes[:, None] * 3 + np.arange(3)).ravel()
    reduced = reduce(scalar, material, fixed, retained, prescribed)
    render_basis, render_distance = visual_basis(
        positions, mesh, scalar, reduced["basis"], faces, face_cells, cell_size
    )
    n = len(control_free) * 3
    control_basis = np.zeros((len(control) * 3, n))
    control_basis[(control_free[:, None] * 3 + np.arange(3)).ravel()] = np.eye(n)
    result = {
        "version": 1,
        "material": material,
        "positions": np.vstack([nodes, control]).ravel().tolist(),
        "triangles": (coarse_triangles + len(nodes)).ravel().tolist(),
        "mount": np.r_[fixed_nodes, control_fixed + len(nodes)].tolist(),
        "coordinates": (control_free + len(nodes)).tolist(),
        "basis": np.vstack([reduced["basis"], control_basis]).ravel().tolist(),
        "renderPositions": positions.ravel().tolist(),
        "renderTriangles": triangles.ravel().tolist(),
        "renderBasis": render_basis.ravel().tolist(),
        "contactRegion": request["contact"],
        "mountRegion": request["mount"],
        "surfaceError": {"contact": contact_distance, "render": render_distance},
    }
    for key in ["mass", "stiffness", "gravity"]:
        result[key] = reduced[key].ravel().tolist()
    if "quadrature" in reduced:
        result["quadrature"] = {key: value.ravel().tolist() for key, value in reduced["quadrature"].items()}
    return result
