import pyCubbyFlow
from pytest import approx
from pytest_utils import assert_vector_similar


def _triangle_mesh():
    return pyCubbyFlow.TriangleMesh3(
        points=[(0, 0, 0), (1, 0, 0), (0, 1, 0)],
        normals=[(0, 0, 1)] * 3,
        uvs=[(0, 0), (1, 0), (0, 1)],
        pointIndices=[(0, 1, 2)],
        normalIndices=[(0, 1, 2)],
        uvIndices=[(0, 1, 2)],
    )


def test_triangle_mesh3_data_access():
    mesh = _triangle_mesh()

    assert mesh.Area() == approx(0.5)
    assert mesh.Volume() == approx(0)
    assert mesh.NumberOfPoints() == 3
    assert mesh.NumberOfNormals() == 3
    assert mesh.NumberOfUVs() == 3
    assert mesh.NumberOfTriangles() == 1
    assert mesh.HasNormals()
    assert mesh.HasUVs()
    assert_vector_similar(mesh.GetPoint(1), (1, 0, 0))
    assert_vector_similar(mesh.GetNormal(0), (0, 0, 1))
    assert tuple(mesh.GetPointIndex(0)) == (0, 1, 2)
    assert tuple(mesh.GetNormalIndex(0)) == (0, 1, 2)
    assert tuple(mesh.GetUVIndex(0)) == (0, 1, 2)
    assert mesh.Triangle(0)

    mesh.SetPoint(1, pyCubbyFlow.Vector3D(2, 0, 0))
    mesh.SetNormal(0, pyCubbyFlow.Vector3D(0, 1, 0))
    mesh.SetPointIndex(0, pyCubbyFlow.Vector3UZ(2, 1, 0))
    mesh.SetNormalIndex(0, pyCubbyFlow.Vector3UZ(2, 1, 0))
    mesh.SetUVIndex(0, pyCubbyFlow.Vector3UZ(2, 1, 0))

    assert_vector_similar(mesh.GetPoint(1), (2, 0, 0))
    assert_vector_similar(mesh.GetNormal(0), (0, 1, 0))
    assert tuple(mesh.GetPointIndex(0)) == (2, 1, 0)
    assert tuple(mesh.GetNormalIndex(0)) == (2, 1, 0)
    assert tuple(mesh.GetUVIndex(0)) == (2, 1, 0)


def test_triangle_mesh3_operations(tmp_path):
    mesh = _triangle_mesh()
    mesh.Scale(2)
    mesh.Translate((1, 2, 3))
    mesh.Rotate(pyCubbyFlow.QuaternionD())
    assert_vector_similar(mesh.GetPoint(1), (3, 2, 3))

    copied = pyCubbyFlow.TriangleMesh3()
    copied.Set(mesh)
    empty = pyCubbyFlow.TriangleMesh3()
    copied.Swap(empty)
    assert copied.NumberOfTriangles() == 0
    assert empty.NumberOfTriangles() == 1

    generated = pyCubbyFlow.TriangleMesh3()
    for point in ((0, 0, 0), (1, 0, 0), (0, 1, 0)):
        generated.AddPoint(point)
    for normal in ((0, 0, 1),) * 3:
        generated.AddNormal(normal)
    for uv in ((0, 0), (1, 0), (0, 1)):
        generated.AddUV(uv)
    generated.AddPointTriangle((0, 1, 2))
    generated.AddNormalTriangle((0, 1, 2))
    generated.AddUVTriangle((0, 1, 2))
    generated.SetFaceNormal()
    generated.SetAngleWeightedVertexNormal()
    assert generated.NumberOfNormals() == 3

    obj = tmp_path / "triangle.obj"
    generated.WriteObj(str(obj))
    loaded = pyCubbyFlow.TriangleMesh3()
    loaded.ReadObj(str(obj))
    assert loaded.NumberOfTriangles() == 1

    generated.Clear()
    assert generated.NumberOfPoints() == 0
