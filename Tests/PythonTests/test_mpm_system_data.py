import numpy as np
import pytest

import pyCubbyFlow


@pytest.mark.parametrize(
    "data_type,resolution,spacing,origin,grid_velocity",
    [
        (
            pyCubbyFlow.MPMSystemData2,
            pyCubbyFlow.Vector2UZ(3, 3),
            pyCubbyFlow.Vector2D(1.0, 1.0),
            pyCubbyFlow.Vector2D(),
            (1.0, 1.0),
        ),
        (
            pyCubbyFlow.MPMSystemData3,
            pyCubbyFlow.Vector3UZ(3, 3, 3),
            pyCubbyFlow.Vector3D(1.0, 1.0, 1.0),
            pyCubbyFlow.Vector3D(),
            (1.0, 1.0, 1.0),
        ),
    ],
)
def test_mpm_system_data_api(
    data_type, resolution, spacing, origin, grid_velocity
):
    data = data_type(resolution, spacing, origin, 1)

    masses = np.asarray(data.particleMasses)
    volumes = np.asarray(data.initialVolumes)
    assert masses.shape == (1,)
    assert volumes.shape == (1,)
    masses[0] = 2.0
    volumes[0] = 3.0
    assert data.particleMasses[0] == 2.0
    assert data.initialVolumes[0] == 3.0

    states = data.deformationStates
    assert len(states) == 1
    states[0].elastic[0, 0] = 2.0
    states[0].plastic[0, 0] = 3.0
    assert states[0].elastic[0, 0] == 2.0
    assert states[0].plastic[0, 0] == 3.0

    assert data.gridMass.resolution == resolution
    assert data.gridVelocities.resolution == resolution
    assert data.gridVelocitiesBeforeUpdate.resolution == resolution

    data.flipBlendingFactor = 0.5
    assert data.flipBlendingFactor == 0.5
    data.TransferFromParticlesToGrid()
    data.gridVelocities.Fill(grid_velocity)
    data.TransferFromGridToParticles()
    resized_grid = resolution + resolution, spacing + spacing, origin + spacing
    data.ResizeGrid(*resized_grid)
    for grid in (
        data.gridMass,
        data.gridVelocities,
        data.gridVelocitiesBeforeUpdate,
    ):
        assert grid.resolution == resized_grid[0]
        assert grid.gridSpacing == resized_grid[1]
        assert grid.gridOrigin == resized_grid[2]


@pytest.mark.parametrize(
    "data_type,resolution,spacing,origin,grid_velocity,dimension",
    [
        (
            pyCubbyFlow.MPMFluidSystemData2,
            pyCubbyFlow.Vector2UZ(3, 3),
            pyCubbyFlow.Vector2D(1.0, 1.0),
            pyCubbyFlow.Vector2D(),
            (1.0, 1.0),
            2,
        ),
        (
            pyCubbyFlow.MPMFluidSystemData3,
            pyCubbyFlow.Vector3UZ(3, 3, 3),
            pyCubbyFlow.Vector3D(1.0, 1.0, 1.0),
            pyCubbyFlow.Vector3D(),
            (1.0, 1.0, 1.0),
            3,
        ),
    ],
)
def test_mpm_fluid_system_data_api(
    data_type,
    resolution,
    spacing,
    origin,
    grid_velocity,
    dimension,
):
    data = data_type(resolution, spacing, origin, 1)

    ratios = np.asarray(data.volumeRatios)
    gradients = data.velocityGradients
    assert ratios.shape == (1,)
    assert len(gradients) == 1
    assert gradients[0].shape == (dimension, dimension)
    assert not hasattr(data, "deformationStates")

    ratios[0] = 2.0
    gradients[0][0, 0] = 3.0
    assert data.volumeRatios[0] == 2.0
    assert data.velocityGradients[0][0, 0] == 3.0
    ratios[0] = 1.0
    gradients[0][0, 0] = 0.0

    data.TransferFromParticlesToGrid()
    data.gridVelocities.Fill(grid_velocity)
    data.TransferFromGridToParticles(0.1)

    assert data.volumeRatios[0] == pytest.approx(1.0)
    assert np.asarray(data.velocityGradients[0]) == pytest.approx(
        np.zeros((dimension, dimension))
    )
