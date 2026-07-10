import pyCubbyFlow
from pytest_utils import *


def test_marching_cubes_connectivity():
    grid = pyCubbyFlow.VertexCenteredScalarGrid3((1, 1, 1))
    grid[(0, 0, 0)] = -0.5
    grid[(0, 0, 1)] = -0.5
    grid[(0, 1, 0)] = 0.5
    grid[(0, 1, 1)] = 0.5
    grid[(1, 0, 0)] = -0.5
    grid[(1, 0, 1)] = -0.5
    grid[(1, 1, 0)] = 0.5
    grid[(1, 1, 1)] = 0.5

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_NONE)
    assert mesh.NumberOfPoints() == 24

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_BACK)
    assert mesh.NumberOfPoints() == 22

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_FRONT)
    assert mesh.NumberOfPoints() == 22

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_LEFT)
    assert mesh.NumberOfPoints() == 22

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_RIGHT)
    assert mesh.NumberOfPoints() == 22

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_DOWN)
    assert mesh.NumberOfPoints() == 24

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_UP)
    assert mesh.NumberOfPoints() == 24

    mesh = pyCubbyFlow.MarchingCubes(
        grid, (1, 1, 1), (0, 0, 0), 0.0, pyCubbyFlow.DIRECTION_ALL, pyCubbyFlow.DIRECTION_ALL)
    assert mesh.NumberOfPoints() == 8
