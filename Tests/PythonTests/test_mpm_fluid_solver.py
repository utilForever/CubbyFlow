import pytest

import pyCubbyFlow


@pytest.mark.parametrize(
    "solver_type,data_type,resolution,spacing,origin,position,gravity",
    [
        (
            pyCubbyFlow.MPMFluidSolver2,
            pyCubbyFlow.MPMFluidSystemData2,
            pyCubbyFlow.Vector2UZ(8, 8),
            pyCubbyFlow.Vector2D(0.1, 0.1),
            pyCubbyFlow.Vector2D(),
            pyCubbyFlow.Vector2D(0.35, 0.35),
            pyCubbyFlow.Vector2D(0.0, -2.0),
        ),
        (
            pyCubbyFlow.MPMFluidSolver3,
            pyCubbyFlow.MPMFluidSystemData3,
            pyCubbyFlow.Vector3UZ(8, 8, 8),
            pyCubbyFlow.Vector3D(0.1, 0.1, 0.1),
            pyCubbyFlow.Vector3D(),
            pyCubbyFlow.Vector3D(0.35, 0.35, 0.35),
            pyCubbyFlow.Vector3D(0.0, -2.0, 0.0),
        ),
    ],
)
def test_mpm_fluid_solver_api(
    solver_type, data_type, resolution, spacing, origin, position, gravity
):
    solver = solver_type(
        resolution,
        spacing,
        origin,
        0.01,
        1.0,
        900.0,
        20.0,
        5.0,
        0.25,
    )

    assert isinstance(solver.mpmSystemData, data_type)
    assert solver.targetDensity == pytest.approx(900.0)
    assert solver.speedOfSound == pytest.approx(20.0)
    assert solver.eosExponent == pytest.approx(5.0)
    assert solver.negativePressureScale == pytest.approx(0.25)
    assert solver.timeStepLimitScale == pytest.approx(0.9)

    solver.timeStepLimitScale = 0.5
    assert solver.timeStepLimitScale == pytest.approx(0.5)
    with pytest.raises(AttributeError):
        solver.targetDensity = 1000.0

    solver.Update(pyCubbyFlow.Frame(0, 0.001))
    solver.gravity = gravity
    solver.dragCoefficient = 0.0
    solver.mpmSystemData.AddParticle(position)
    solver.Update(pyCubbyFlow.Frame(1, 0.001))

    velocity = solver.mpmSystemData.velocities[0]
    assert velocity.y == pytest.approx(-0.002, abs=1e-10)
