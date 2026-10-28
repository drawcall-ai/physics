"""A coarse contact skin and its interpolation onto the FEM boundary."""

import geometry
import numpy as np
import trimesh
from pyodide.ffi import to_js

from surface import closest


def bounds(region):
    low, high = np.asarray(region["min"]), np.asarray(region["max"])
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(low > high):
        raise ValueError("FEM regions require finite ordered min/max bounds")
    return low, high


def inside(points, region):
    low, high = bounds(region)
    return ((points >= low) & (points <= high)).all(axis=1)


def overlaps(points, triangles, region):
    low, high = bounds(region)
    vertices = points[triangles]
    # Keep whole intersecting triangles; the runtime guard enforces exact authored bounds.
    return ((vertices.max(axis=1) >= low) & (vertices.min(axis=1) <= high)).all(axis=1)


def coarse_surface(positions, triangles, max_dofs):
    indices, inverse = np.unique(triangles, return_inverse=True)
    points = positions[indices]
    faces = inverse.reshape(-1, 3)
    if max_dofs < 9:
        raise ValueError("FEM maxDofs must allow at least three contact vertices (9 DOFs)")
    while len(points) * 3 > max_dofs:
        target = max(1, min(len(faces) - 1, int(len(faces) * max_dofs / (len(points) * 3) * .8)))
        reduced = geometry.simplify(
            to_js(np.asarray(points, dtype=np.float64).ravel()),
            to_js(np.asarray(faces, dtype=np.uint32).ravel()),
            target,
        )
        vertices = np.asarray(reduced.positions.to_py()).reshape(-1, 3)
        cells = np.asarray(reduced.triangles.to_py()).reshape(-1, 3)
        if not len(cells) or len(vertices) >= len(points):
            raise ValueError("Contact surface cannot fit maxDofs; increase the coordinate budget")
        points, faces = vertices, cells
    surface = trimesh.Trimesh(vertices=points, faces=faces, process=False)
    if not surface.is_winding_consistent or np.any(surface.area_faces <= 0):
        raise ValueError("Contact simplification produced invalid triangles; increase maxDofs")
    return points, faces


def contact_surface(nodes, faces, positions, source_faces, max_dofs, mount):
    boundary = nodes[np.unique(faces)]
    if len(boundary) * 3 <= max_dofs:
        return coarse_surface(nodes, faces, max_dofs)
    if len(source_faces):
        _, distances, _ = closest(positions, source_faces, boundary)
        # Prefer an exact authored patch, refining it only while its coordinates stay independent.
        if distances.max() < np.ptp(positions, axis=0).max() * 1e-8:
            control, triangles = coarse_surface(positions, source_faces, max_dofs)
            if independent(boundary, control, triangles, mount):
                while True:
                    points, cells = trimesh.remesh.subdivide(control, triangles)
                    if len(points) * 3 > max_dofs or not independent(boundary, points, cells, mount):
                        break
                    control, triangles = points, cells
                return control, triangles
    return coarse_surface(nodes, faces, max_dofs)


def independent(boundary, control, triangles, mount):
    weights, _ = interpolation(boundary, control, triangles)
    free = ~inside(control, mount)
    return np.any(free) and np.linalg.matrix_rank(weights[:, free]) == np.count_nonzero(free)


def interpolation(points, control, triangles):
    nearest, distances, faces = closest(control, triangles, points)
    weights = trimesh.triangles.points_to_barycentric(control[triangles[faces]], nearest)
    if not np.isfinite(weights).all() or np.any(weights < -1e-8):
        raise ValueError("Invalid contact-surface interpolation")
    matrix = np.zeros((len(points), len(control)))
    for corner in range(3):
        matrix[np.arange(len(points)), triangles[faces, corner]] += weights[:, corner]
    return matrix, float(distances.max())
