import pytest

import pyCubbyFlow


@pytest.mark.parametrize(
    "solver_type,data_type,resolution,spacing,origin",
    [
        (
            pyCubbyFlow.SnowMPMSolver2,
            pyCubbyFlow.MPMSystemData2,
            pyCubbyFlow.Vector2UZ(4, 4),
            pyCubbyFlow.Vector2D(0.5, 1.0),
            pyCubbyFlow.Vector2D(-1.0, -2.0),
        ),
        (
            pyCubbyFlow.SnowMPMSolver3,
            pyCubbyFlow.MPMSystemData3,
            pyCubbyFlow.Vector3UZ(4, 4, 4),
            pyCubbyFlow.Vector3D(0.5, 1.0, 2.0),
            pyCubbyFlow.Vector3D(-1.0, -2.0, -3.0),
        ),
    ],
)
def test_snow_mpm_solver_api(
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

    solver.Update(pyCubbyFlow.Frame(0, 0.001))
