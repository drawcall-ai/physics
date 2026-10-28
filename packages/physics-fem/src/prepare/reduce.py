"""Fixed-interface static condensation with projected FEM mass and material quadrature."""

import numpy as np
from scipy.sparse.linalg import splu
from skfem import Basis, BilinearForm, ElementTetP1, ElementTetP2, ElementVector, asm
from skfem.helpers import dot
from skfem.models.elasticity import lame_parameters, linear_elasticity


@BilinearForm
def mass(u, v, w):
    return dot(u, v)


def discretize(mesh, order):
    if order not in [1, 2]:
        raise ValueError("FEM mesh order must be 1 or 2")
    element = ElementTetP1() if order == 1 else ElementTetP2()
    return Basis(mesh, element, intorder=2 if order == 1 else 5)


def reduce(scalar, material, fixed, retained, prescribed):
    elastic = material["elasticity"]
    density = material["density"]
    young, poisson = elastic["youngModulus"], elastic["poissonRatio"]
    if not np.isfinite([density, young, poisson]).all():
        raise ValueError("FEM material constants must be finite")
    if density <= 0 or young <= 0 or not 0 <= poisson < .5:
        raise ValueError("Invalid FEM density or elastic constants")
    if elastic["model"] not in ["linear", "neoHookean", "saintVenantKirchhoff"]:
        raise ValueError("Unsupported reduced FEM material")
    vector = Basis(scalar.mesh, ElementVector(scalar.elem), quadrature=scalar.quadrature)
    fixed, retained = np.asarray(fixed), np.asarray(retained)
    if fixed.ndim != 1 or retained.ndim != 1:
        raise ValueError("FEM boundary DOFs must be one-dimensional")
    selected = np.r_[fixed, retained]
    integral = np.issubdtype(selected.dtype, np.integer)
    if not integral or np.any(selected < 0) or np.any(selected >= vector.N):
        raise ValueError("FEM boundary DOFs must be valid integer indices")
    if not len(fixed) or len(np.unique(selected)) != len(selected):
        raise ValueError("FEM boundary DOFs must be disjoint and the mount nonempty")
    prescribed = np.asarray(prescribed, dtype=float)
    if prescribed.ndim != 2 or prescribed.shape[0] != len(retained):
        raise ValueError("Invalid FEM contact interpolation")
    if not np.isfinite(prescribed).all():
        raise ValueError("FEM contact interpolation must be finite")
    n = prescribed.shape[1]
    if not n or n % 3 or np.linalg.matrix_rank(prescribed) != n:
        raise ValueError("FEM contact coordinates must be independent XYZ translations")
    interior = np.setdiff1d(np.arange(vector.N), selected)
    stiffness = asm(linear_elasticity(*lame_parameters(young, poisson)), vector).tocsc()
    inertia = (density * asm(mass, vector)).tocsc()
    basis = np.zeros((vector.N, n))
    basis[retained] = prescribed
    if len(interior):
        factor = splu(stiffness[interior][:, interior])
        basis[interior] = -factor.solve(stiffness[interior][:, retained] @ prescribed)
    reduced_k = basis.T @ (stiffness @ basis)
    rigid = np.tile(np.eye(3), (scalar.N, 1))
    modes = np.column_stack([rigid, basis])
    reduced_m = modes.T @ (inertia @ modes)
    reduced_k = (reduced_k + reduced_k.T) / 2
    reduced_m = (reduced_m + reduced_m.T) / 2
    for label, matrix in [("mass", reduced_m), ("stiffness", reduced_k)]:
        if not np.isfinite(matrix).all():
            raise ValueError(f"Non-finite reduced FEM {label}")
        try:
            np.linalg.cholesky(matrix)
        except np.linalg.LinAlgError as error:
            raise ValueError(
                f"Reduced FEM {label} is not positive definite; check mounting and mesh quality"
            ) from error
    result = {
        "basis": basis,
        "mass": reduced_m,
        "stiffness": reduced_k,
        "gravity": reduced_m[:, :3],
    }
    if elastic["model"] == "linear":
        return result
    energy = scalar
    if isinstance(scalar.elem, ElementTetP1):
        energy = Basis(scalar.mesh, scalar.elem, intorder=1)
    reduced = basis.reshape(scalar.N, 3, n)
    gradients = np.zeros((*energy.dx.shape, 3, 3, n))
    for local, fields in enumerate(energy.basis):
        values = reduced[energy.element_dofs[local]]
        gradients += np.einsum("jen,eir->enijr", fields[0].grad, values)
    active = np.any(gradients != 0, axis=(2, 3, 4))
    weights, gradients = energy.dx[active], gradients[active]
    if not len(weights) or np.any(weights <= 0):
        raise ValueError("Invalid reduced FEM material quadrature")
    if not np.isfinite(weights).all() or not np.isfinite(gradients).all():
        raise ValueError("FEM material quadrature must be finite")
    result["quadrature"] = {"weights": weights, "gradients": gradients}
    return result
