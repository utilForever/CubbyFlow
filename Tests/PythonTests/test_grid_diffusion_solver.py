import pyCubbyFlow
import pytest
from pytest import approx


CASES = [
    (
        pyCubbyFlow.GridForwardEulerDiffusionSolver2,
        pyCubbyFlow.CellCenteredScalarGrid2,
        pyCubbyFlow.CellCenteredVectorGrid2,
        pyCubbyFlow.FaceCenteredGrid2,
        pyCubbyFlow.ConstantScalarField2,
        (2, 2),
        (0.5, 0.5),
        (1.0, 2.0),
    ),
    (
        pyCubbyFlow.GridBackwardEulerDiffusionSolver2,
        pyCubbyFlow.CellCenteredScalarGrid2,
        pyCubbyFlow.CellCenteredVectorGrid2,
        pyCubbyFlow.FaceCenteredGrid2,
        pyCubbyFlow.ConstantScalarField2,
        (2, 2),
        (0.5, 0.5),
        (1.0, 2.0),
    ),
    (
        pyCubbyFlow.GridForwardEulerDiffusionSolver3,
        pyCubbyFlow.CellCenteredScalarGrid3,
        pyCubbyFlow.CellCenteredVectorGrid3,
        pyCubbyFlow.FaceCenteredGrid3,
        pyCubbyFlow.ConstantScalarField3,
        (2, 2, 2),
        (0.5, 0.5, 0.5),
        (1.0, 2.0, 3.0),
    ),
    (
        pyCubbyFlow.GridBackwardEulerDiffusionSolver3,
        pyCubbyFlow.CellCenteredScalarGrid3,
        pyCubbyFlow.CellCenteredVectorGrid3,
        pyCubbyFlow.FaceCenteredGrid3,
        pyCubbyFlow.ConstantScalarField3,
        (2, 2, 2),
        (0.5, 0.5, 0.5),
        (1.0, 2.0, 3.0),
    ),
]


@pytest.mark.parametrize(
    "solver_type,scalar_type,vector_type,face_type,field_type,"
    "resolution,point,value",
    CASES,
)
def test_grid_diffusion_solver_accepts_custom_sdfs(
    solver_type,
    scalar_type,
    vector_type,
    face_type,
    field_type,
    resolution,
    point,
    value,
):
    source = scalar_type(resolution)
    source.Fill(1.0)
    dest = scalar_type(resolution)

    solver_type().Solver(
        source,
        0.1,
        0.1,
        dest,
        boundarySDF=field_type(1.0),
        fluidSDF=field_type(-1.0),
    )

    assert dest.Sample(point) == approx(1.0)


@pytest.mark.parametrize(
    "solver_type,scalar_type,vector_type,face_type,field_type,"
    "resolution,point,value",
    CASES,
)
def test_grid_diffusion_solver_dispatches_grid_types(
    solver_type,
    scalar_type,
    vector_type,
    face_type,
    field_type,
    resolution,
    point,
    value,
):
    solver = solver_type()

    vector = vector_type(resolution)
    vector.Fill(value)
    vector_dest = vector_type(resolution)
    solver.Solver(vector, 0.1, 0.1, vector_dest)
    assert tuple(vector_dest.Sample(point)) == approx(value)

    face = face_type(resolution)
    face.Fill(value)
    face_dest = face_type(resolution)
    solver.Solver(face, 0.1, 0.1, face_dest)
    assert tuple(face_dest.Sample(point)) == approx(value)

    scalar = scalar_type(resolution)
    with pytest.raises(ValueError):
        solver.Solver(scalar, 0.1, 0.1, vector_dest)
