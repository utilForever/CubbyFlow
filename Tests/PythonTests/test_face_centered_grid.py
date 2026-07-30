import pyCubbyFlow
import pytest
from pytest import approx
from pytest_utils import assert_vector_similar


def test_face_centered_grid2_Fill():
    a = pyCubbyFlow.FaceCenteredGrid2((10, 10))
    a.Fill((3.0, 4.0))
    for j in range(10):
        for i in range(11):
            assert a.U((i, j)) == 3.0
    for j in range(11):
        for i in range(10):
            assert a.V((i, j)) == 4.0

    def filler(pt):
        return (pt.x, pt.y)

    a.Fill(filler)
    for j in range(10):
        for i in range(11):
            assert a.U((i, j)) == i
    for j in range(11):
        for i in range(10):
            assert a.V((i, j)) == j


def test_face_centered_grid2_for_each():
    a = pyCubbyFlow.FaceCenteredGrid2((10, 10))
    # Workaround for Python 2.x which doesn't support nonlocal
    d = {'ei': 0, 'ej': 0}

    def checkU(idx):
        assert idx[0] == d['ei']
        assert idx[1] == d['ej']
        d['ei'] += 1
        if d['ei'] >= 11:
            d['ei'] = 0
            d['ej'] += 1

    a.ForEachUIndex(checkU)
    d = {'ei': 0, 'ej': 0}

    def checkV(idx):
        assert idx[0] == d['ei']
        assert idx[1] == d['ej']
        d['ei'] += 1
        if d['ei'] >= 10:
            d['ei'] = 0
            d['ej'] += 1

    a.ForEachVIndex(checkV)


def test_face_centered_grid2_serialization():
    a = pyCubbyFlow.FaceCenteredGrid2((10, 10))

    def filler(pt):
        return (pt.x, pt.y)

    a.Fill(filler)

    flatBuffer = a.Serialize()

    b = pyCubbyFlow.FaceCenteredGrid2()
    b.Deserialize(flatBuffer)

    for j in range(10):
        for i in range(11):
            assert b.U((i, j)) == i
    for j in range(11):
        for i in range(10):
            assert b.V((i, j)) == j


def test_face_centered_grid2_queries():
    grid = pyCubbyFlow.FaceCenteredGrid2((2, 2))
    grid.Fill(pyCubbyFlow.Vector2D(1, 2))
    with pytest.raises(ValueError):
        grid.Fill(1)
    grid.Fill(lambda pt: (pt.x, pt.y))

    assert tuple(grid.USize()) == (3, 2)
    assert tuple(grid.VSize()) == (2, 3)
    assert_vector_similar(grid.UOrigin(), (0, 0.5))
    assert_vector_similar(grid.VOrigin(), (0.5, 0))
    assert_vector_similar(grid.UPosition()((1, 1)), (1, 1.5))
    assert_vector_similar(grid.VPosition()((1, 1)), (1.5, 1))
    assert_vector_similar(grid.ValueAtCellCenter((0, 0)), (0.5, 0.5))
    assert grid.DivergenceAtCellCenter((0, 0)) == approx(2)
    assert grid.CurlAtCellCenter((0, 0)) == approx(0)
    assert_vector_similar(grid.Sample((0.5, 0.5)), (0.5, 0.5))
    assert grid.Divergence((0.5, 0.5)) == approx(2)
    assert grid.Curl((0.5, 0.5)) == approx(0)
    assert_vector_similar(
        grid.Sampler()(pyCubbyFlow.Vector2D(0.5, 0.5)), (0.5, 0.5)
    )
    assert grid.UView()
    assert grid.VView()

    grid.SetU((0, 0), 3)
    grid.SetV((0, 0), 4)
    assert grid.U((0, 0)) == 3
    assert grid.V((0, 0)) == 4

    copied = pyCubbyFlow.FaceCenteredGrid2()
    copied.Set(grid)
    assert copied.U((0, 0)) == 3
    assert copied.V((0, 0)) == 4


def test_face_centered_grid3():
    grid = pyCubbyFlow.FaceCenteredGrid3((2, 2, 2))
    grid.Fill(pyCubbyFlow.Vector3D(1, 2, 3))
    grid.Fill((1, 2, 3))
    with pytest.raises(ValueError):
        grid.Fill(1)
    grid.Fill(lambda pt: (pt.x, pt.y, pt.z))

    assert tuple(grid.USize()) == (3, 2, 2)
    assert tuple(grid.VSize()) == (2, 3, 2)
    assert tuple(grid.WSize()) == (2, 2, 3)
    assert_vector_similar(grid.UOrigin(), (0, 0.5, 0.5))
    assert_vector_similar(grid.VOrigin(), (0.5, 0, 0.5))
    assert_vector_similar(grid.WOrigin(), (0.5, 0.5, 0))
    assert_vector_similar(grid.UPosition()((1, 1, 1)), (1, 1.5, 1.5))
    assert_vector_similar(grid.VPosition()((1, 1, 1)), (1.5, 1, 1.5))
    assert_vector_similar(grid.WPosition()((1, 1, 1)), (1.5, 1.5, 1))
    assert_vector_similar(
        grid.ValueAtCellCenter((0, 0, 0)), (0.5, 0.5, 0.5)
    )
    assert grid.DivergenceAtCellCenter((0, 0, 0)) == approx(3)
    assert_vector_similar(grid.CurlAtCellCenter((0, 0, 0)), (0, 0, 0))
    assert_vector_similar(grid.Sample((0.5, 0.5, 0.5)), (0.5, 0.5, 0.5))
    assert grid.Divergence((0.5, 0.5, 0.5)) == approx(3)
    assert_vector_similar(grid.Curl((0.5, 0.5, 0.5)), (0, 0, 0))
    assert_vector_similar(
        grid.Sampler()(pyCubbyFlow.Vector3D(0.5, 0.5, 0.5)),
        (0.5, 0.5, 0.5),
    )
    assert grid.UView()
    assert grid.VView()
    assert grid.WView()

    u_indices, v_indices, w_indices = [], [], []
    grid.ForEachUIndex(u_indices.append)
    grid.ForEachVIndex(v_indices.append)
    grid.ForEachWIndex(w_indices.append)
    assert len(u_indices) == 12
    assert len(v_indices) == 12
    assert len(w_indices) == 12

    restored = pyCubbyFlow.FaceCenteredGrid3()
    restored.Deserialize(grid.Serialize())
    assert_vector_similar(
        restored.Sample((0.5, 0.5, 0.5)), (0.5, 0.5, 0.5)
    )


def test_face_centered_grid3_set():
    grid = pyCubbyFlow.FaceCenteredGrid3((2, 2, 2))
    grid.SetU((0, 0, 0), 3)
    grid.SetV((0, 0, 0), 4)
    grid.SetW((0, 0, 0), 5)
    assert grid.U((0, 0, 0)) == 3
    assert grid.V((0, 0, 0)) == 4
    assert grid.W((0, 0, 0)) == 5

    copied = pyCubbyFlow.FaceCenteredGrid3()
    copied.Set(grid)
    assert copied.U((0, 0, 0)) == 3
    assert copied.V((0, 0, 0)) == 4
    assert copied.W((0, 0, 0)) == 5
