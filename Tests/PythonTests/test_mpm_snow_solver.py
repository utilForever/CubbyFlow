import pytest

import pyCubbyFlow


@pytest.mark.parametrize(
    "solver_type,data_type,resolution,spacing,origin",
    [
        (
            pyCubbyFlow.MPMSnowSolver2,
            pyCubbyFlow.MPMSystemData2,
            pyCubbyFlow.Vector2UZ(4, 4),
            pyCubbyFlow.Vector2D(0.5, 1.0),
            pyCubbyFlow.Vector2D(-1.0, -2.0),
        ),
        (
            pyCubbyFlow.MPMSnowSolver3,
            pyCubbyFlow.MPMSystemData3,
            pyCubbyFlow.Vector3UZ(4, 4, 4),
            pyCubbyFlow.Vector3D(0.5, 1.0, 2.0),
            pyCubbyFlow.Vector3D(-1.0, -2.0, -3.0),
        ),
    ],
)
def test_mpm_snow_solver_api(
    solver_type, data_type, resolution, spacing, origin
):
    solver = solver_type(resolution, spacing, origin, 0.1, 2.0)

    assert isinstance(solver.mpmSystemData, data_type)
    assert solver.mpmSystemData.gridMass.resolution == resolution
    assert solver.mpmSystemData.gridMass.gridSpacing == spacing
    assert solver.mpmSystemData.gridMass.gridOrigin == origin

    assert solver.timeStepLimitScale == pytest.approx(0.9)
    solver.timeStepLimitScale = 0.5
    assert solver.timeStepLimitScale == pytest.approx(0.5)

    assert solver.closedDomainBoundaryFlag == pyCubbyFlow.DIRECTION_ALL
    solver.closedDomainBoundaryFlag = 5
    assert solver.closedDomainBoundaryFlag == 5

    assert solver.isUsingSemiImplicit is False
    solver.isUsingSemiImplicit = True
    assert solver.isUsingSemiImplicit is True

    assert solver.maxNumberOfIterations == 100
    solver.maxNumberOfIterations = 25
    assert solver.maxNumberOfIterations == 25

    assert solver.tolerance == pytest.approx(1e-6)
    solver.tolerance = 1e-8
    assert solver.tolerance == pytest.approx(1e-8)

    with pytest.raises(ValueError):
        solver.tolerance = 0.0

    assert solver.lastNumberOfIterations == 0
    assert solver.lastResidual == pytest.approx(0.0)

    solver.Update(pyCubbyFlow.Frame(0, 0.001))
