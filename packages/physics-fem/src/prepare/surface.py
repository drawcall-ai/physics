"""FEM boundary topology and interpolation onto the authored render surface."""

import geometry
import numpy as np
from pyodide.ffi import to_js
from skfem import ElementTetP1


def closest(positions, triangles, points):
    result = geometry.closest(
        to_js(np.asarray(positions, dtype=np.float64).ravel()),
        to_js(np.asarray(triangles, dtype=np.uint32).ravel()),
        to_js(np.asarray(points, dtype=np.float64).ravel()),
    )
    return (
        np.asarray(result.points.to_py()).reshape(-1, 3),
        np.asarray(result.distances.to_py()),
        np.asarray(result.faces.to_py()),
    )


def boundary(mesh, scalar):
    facets = mesh.boundary_facets()
    triangles = mesh.facets[:, facets].T.copy()
    cells = mesh.f2t[0, facets]
    positions = mesh.p.T
    a, b, c = (positions[triangles[:, i]] for i in range(3))
    normal = np.cross(b - a, c - a)
    inward = (normal * (positions[mesh.t[:, cells]].mean(axis=0) - a)).sum(axis=1) > 0
    triangles[inward, 1], triangles[inward, 2] = triangles[inward, 2].copy(), triangles[inward, 1].copy()
    if isinstance(scalar.elem, ElementTetP1):
        return triangles, cells
    edges = {tuple(pair): int(scalar.edge_dofs[0, i]) for i, pair in enumerate(mesh.edges.T)}
    refined = []
    for a, b, c in triangles:
        ab, bc, ca = (edges[tuple(sorted(pair))] for pair in [(a, b), (b, c), (c, a)])
        refined.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
    return np.asarray(refined), np.repeat(cells, 4)


def visual_basis(positions, mesh, scalar, basis, triangles, cells, cell_size):
    nearest, distances, faces = closest(scalar.doflocs.T, triangles, positions)
    if distances.max() > cell_size / 2:
        raise ValueError("FEM mesh deviates from the rendered surface by more than half cellSize; refine the mesh")
    elements = cells[faces]
    corners = mesh.p.T[mesh.t.T[elements]]
    edges = (corners[:, 1:] - corners[:, :1]).transpose(0, 2, 1)
    reference = np.linalg.solve(edges, (nearest - corners[:, 0])[..., None])[..., 0]
    reduced = basis.reshape(scalar.N, 3, -1)
    result = np.zeros((len(positions), 3, reduced.shape[2]))
    for local in range(scalar.element_dofs.shape[0]):
        values, _ = scalar.elem.lbasis(reference.T, local)
        nodes = scalar.element_dofs[local, elements]
        result += values[:, None, None] * reduced[nodes]
    return result, float(distances.max())
