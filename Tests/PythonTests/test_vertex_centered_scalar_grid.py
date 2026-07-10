import numpy as np
import pyCubbyFlow
from pytest import approx
from pytest_utils import *

cnt = 0


def test_grid2():
    global cnt

    a = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), gridSpacing=(1, 2), gridOrigin=(7, 5)
    )

    assert a.resolution == (3, 4)
    assert_vector_similar(a.gridOrigin, (7, 5))
    assert_vector_similar(a.gridSpacing, (1, 2))
    assert_bounding_box_similar(
        a.boundingBox, pyCubbyFlow.BoundingBox2D((7, 5), (10, 13))
    )
    f = a.cellCenterPosition
    assert_vector_similar(f(0, 0), (7.5, 6))

    b = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), gridSpacing=(1, 2), gridOrigin=(7, 5)
    )
    assert a.HasSameShape(b)

    def func(idx):
        global cnt
        assert idx[0] >= 0 and idx[0] < 3
        assert idx[1] >= 0 and idx[1] < 4
        cnt += 1

    cnt = 0
    a.ForEachCellIndex(func)
    assert cnt == 12


def test_scalar_grid2():
    global cnt
    a = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), gridSpacing=(1, 2), gridOrigin=(7, 5)
    )

    a.Resize(resolution=(12, 7), gridSpacing=(3, 4), gridOrigin=(9, 2))
    assert a.resolution == (12, 7)
    assert_vector_similar(a.gridOrigin, (9, 2))
    assert_vector_similar(a.gridSpacing, (3, 4))

    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == 0.0

    a[5, 6] = 17.0
    assert a[5, 6] == 17.0

    a.Fill(42.0)
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == 42.0

    def func(pt):
        return pt.x**2 + pt.y**2

    a.Fill(func)
    pos = a.DataPosition()
    acc = np.array(a.DataView(), copy=False)
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            pt = pos(i, j)
            assert func(pt) == a[i, j]
            assert func(pt) == approx(a.Sample(pt))
            assert acc[j, i] == a[i, j]
            # Can't compare to analytic solution because FDM with such a coarse
            # grid will return inaccurate results by design.
            assert_vector_similar(a.GradientAtDataPoint((i, j)), a.Gradient(pt))
            assert a.LaplacianAtDataPoint((i, j)) == a.Laplacian(pt)

    def index_func(idx):
        global cnt
        assert idx[0] >= 0 and idx[0] < a.resolution.x + 1
        assert idx[1] >= 0 and idx[1] < a.resolution.y + 1
        cnt += 1

    cnt = 0
    a.ForEachDataPointIndex(index_func)
    assert cnt == (a.resolution.x + 1) * (a.resolution.y + 1)

    blob = a.Serialize()
    b = pyCubbyFlow.VertexCenteredScalarGrid2()
    b.Deserialize(blob)
    assert b.resolution == (12, 7)
    assert_vector_similar(b.gridOrigin, (9, 2))
    assert_vector_similar(b.gridSpacing, (3, 4))
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == b[i, j]


def test_cell_centered_scalar_grid2():
    # CTOR
    a = pyCubbyFlow.VertexCenteredScalarGrid2()
    assert a.resolution == (1, 1)
    assert_vector_similar(a.gridOrigin, (0.0, 0.0))
    assert_vector_similar(a.gridSpacing, (1.0, 1.0))

    a = pyCubbyFlow.VertexCenteredScalarGrid2((3, 4), (1, 2), (7, 5))
    assert a.resolution == (3, 4)
    assert_vector_similar(a.gridOrigin, (7, 5))
    assert_vector_similar(a.gridSpacing, (1, 2))

    a = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), gridSpacing=(1, 2), gridOrigin=(7, 5)
    )
    assert a.resolution == (3, 4)
    assert_vector_similar(a.gridOrigin, (7, 5))
    assert_vector_similar(a.gridSpacing, (1, 2))

    a = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), domainSizeX=12.0, gridOrigin=(7, 5)
    )
    assert a.resolution == (3, 4)
    assert_vector_similar(a.gridOrigin, (7, 5))
    assert_vector_similar(a.gridSpacing, (4, 4))

    # Properties
    a = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(3, 4), gridSpacing=(1, 2), gridOrigin=(7, 5)
    )
    assert_vector_similar(a.dataSize, (4, 5))
    assert_vector_similar(a.dataOrigin, (7, 5))

    # Modifiers
    b = pyCubbyFlow.VertexCenteredScalarGrid2(
        resolution=(6, 3), gridSpacing=(5, 9), gridOrigin=(1, 2)
    )
    a.Fill(42.0)
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == 42.0

    a.Swap(b)
    assert a.resolution == (6, 3)
    assert_vector_similar(a.gridOrigin, (1, 2))
    assert_vector_similar(a.gridSpacing, (5, 9))
    assert b.resolution == (3, 4)
    assert_vector_similar(b.gridOrigin, (7, 5))
    assert_vector_similar(b.gridSpacing, (1, 2))
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == 0.0
    for j in range(b.resolution.y):
        for i in range(b.resolution.x):
            assert b[i, j] == 42.0

    a.Set(b)
    assert a.resolution == (3, 4)
    assert_vector_similar(a.gridOrigin, (7, 5))
    assert_vector_similar(a.gridSpacing, (1, 2))
    for j in range(a.resolution.y):
        for i in range(a.resolution.x):
            assert a[i, j] == 42.0

    c = a.Clone()
    assert c.resolution == (3, 4)
    assert_vector_similar(c.gridOrigin, (7, 5))
    assert_vector_similar(c.gridSpacing, (1, 2))
    for j in range(c.resolution.y):
        for i in range(c.resolution.x):
            assert c[i, j] == 42.0


# ------------------------------------------------------------------------------


def test_grid3():
    global cnt

    a = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), gridSpacing=(1, 2, 3), gridOrigin=(7, 5, 3)
    )

    assert a.resolution == (3, 4, 5)
    assert_vector_similar(a.gridOrigin, (7, 5, 3))
    assert_vector_similar(a.gridSpacing, (1, 2, 3))
    assert_bounding_box_similar(
        a.boundingBox, pyCubbyFlow.BoundingBox3D((7, 5, 3), (10, 13, 18))
    )
    f = a.cellCenterPosition
    assert_vector_similar(f(0, 0, 0), (7.5, 6, 4.5))

    b = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), gridSpacing=(1, 2, 3), gridOrigin=(7, 5, 3)
    )
    assert a.HasSameShape(b)

    def func(idx):
        global cnt
        assert idx[0] >= 0 and idx[0] < 3
        assert idx[1] >= 0 and idx[1] < 4
        assert idx[2] >= 0 and idx[2] < 5
        cnt += 1

    cnt = 0
    a.ForEachCellIndex(func)
    assert cnt == 60


def test_scalar_grid3():
    global cnt
    a = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), gridSpacing=(1, 2, 3), gridOrigin=(7, 5, 3)
    )

    a.Resize(resolution=(12, 7, 2), gridSpacing=(3, 4, 5), gridOrigin=(9, 2, 5))
    assert a.resolution == (12, 7, 2)
    assert_vector_similar(a.gridOrigin, (9, 2, 5))
    assert_vector_similar(a.gridSpacing, (3, 4, 5))

    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == 0.0

    a[5, 6, 1] = 17.0
    assert a[5, 6, 1] == 17.0

    a.Fill(42.0)
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == 42.0

    def func(pt):
        return pt.x**2 + pt.y**2 + pt.z**2

    a.Fill(func)
    pos = a.DataPosition()
    acc = np.array(a.DataView(), copy=False)
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                pt = pos(i, j, k)
                assert func(pt) == a[i, j, k]
                assert func(pt) == approx(a.Sample(pt))
                assert acc[k, j, i] == a[i, j, k]
                # Can't compare to analytic solution because FDM with such a
                # coarse grid will return inaccurate results by design.
                assert_vector_similar(a.GradientAtDataPoint((i, j, k)), a.Gradient(pt))
                assert a.LaplacianAtDataPoint((i, j, k)) == a.Laplacian(pt)

    def func(idx):
        global cnt
        assert idx[0] >= 0 and idx[0] < a.resolution.x
        assert idx[1] >= 0 and idx[1] < a.resolution.y
        assert idx[2] >= 0 and idx[2] < a.resolution.z
        cnt += 1

    cnt = 0
    a.ForEachDataPointIndex(func)
    assert cnt == a.resolution.x * a.resolution.y * a.resolution.z

    blob = a.Serialize()
    b = pyCubbyFlow.CellCenteredScalarGrid3()
    b.Deserialize(blob)
    assert b.resolution == (12, 7, 2)
    assert_vector_similar(b.gridOrigin, (9, 2, 5))
    assert_vector_similar(b.gridSpacing, (3, 4, 5))
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == b[i, j, k]


def test_cell_centered_scalar_grid3():
    # CTOR
    a = pyCubbyFlow.CellCenteredScalarGrid3()
    assert a.resolution == (1, 1, 1)
    assert_vector_similar(a.gridOrigin, (0.0, 0.0, 0.0))
    assert_vector_similar(a.gridSpacing, (1.0, 1.0, 1.0))

    a = pyCubbyFlow.CellCenteredScalarGrid3((3, 4, 5), (1, 2, 3), (7, 5, 2))
    assert a.resolution == (3, 4, 5)
    assert_vector_similar(a.gridOrigin, (7, 5, 2))
    assert_vector_similar(a.gridSpacing, (1, 2, 3))

    a = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), gridSpacing=(1, 2, 3), gridOrigin=(7, 5, 2)
    )
    assert a.resolution == (3, 4, 5)
    assert_vector_similar(a.gridOrigin, (7, 5, 2))
    assert_vector_similar(a.gridSpacing, (1, 2, 3))

    a = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), domainSizeX=12.0, gridOrigin=(7, 5, 2)
    )
    assert a.resolution == (3, 4, 5)
    assert_vector_similar(a.gridOrigin, (7, 5, 2))
    assert_vector_similar(a.gridSpacing, (4, 4, 4))

    # Properties
    a = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(3, 4, 5), gridSpacing=(1, 2, 3), gridOrigin=(7, 5, 2)
    )
    assert_vector_similar(a.dataSize, (3, 4, 5))
    assert_vector_similar(a.dataOrigin, (7.5, 6, 3.5))

    # Modifiers
    b = pyCubbyFlow.CellCenteredScalarGrid3(
        resolution=(6, 3, 7), gridSpacing=(5, 9, 3), gridOrigin=(1, 2, 8)
    )
    a.Fill(42.0)
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == 42.0

    a.Swap(b)
    assert a.resolution == (6, 3, 7)
    assert_vector_similar(a.gridOrigin, (1, 2, 8))
    assert_vector_similar(a.gridSpacing, (5, 9, 3))
    assert b.resolution == (3, 4, 5)
    assert_vector_similar(b.gridOrigin, (7, 5, 2))
    assert_vector_similar(b.gridSpacing, (1, 2, 3))
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == 0.0
    for k in range(b.resolution.z):
        for j in range(b.resolution.y):
            for i in range(b.resolution.x):
                assert b[i, j, k] == 42.0

    a.Set(b)
    assert a.resolution == (3, 4, 5)
    assert_vector_similar(a.gridOrigin, (7, 5, 2))
    assert_vector_similar(a.gridSpacing, (1, 2, 3))
    for k in range(a.resolution.z):
        for j in range(a.resolution.y):
            for i in range(a.resolution.x):
                assert a[i, j, k] == 42.0

    c = a.Clone()
    assert c.resolution == (3, 4, 5)
    assert_vector_similar(c.gridOrigin, (7, 5, 2))
    assert_vector_similar(c.gridSpacing, (1, 2, 3))
    for k in range(c.resolution.z):
        for j in range(c.resolution.y):
            for i in range(c.resolution.x):
                assert c[i, j, k] == 42.0
