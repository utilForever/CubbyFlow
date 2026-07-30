import pyCubbyFlow
from pytest_utils import assert_vector_similar


def test_constant_scalar_field2():
    field = pyCubbyFlow.ConstantScalarField2(3.5)

    assert field.Sample((1, 2)) == 3.5
    assert field.Sampler()(pyCubbyFlow.Vector2D(3, 4)) == 3.5


def test_constant_scalar_field3():
    field = pyCubbyFlow.ConstantScalarField3(4.5)

    assert field.Sample((1, 2, 3)) == 4.5
    assert field.Sampler()(pyCubbyFlow.Vector3D(3, 4, 5)) == 4.5


def test_constant_vector_field2():
    field = pyCubbyFlow.ConstantVectorField2(pyCubbyFlow.Vector2D(1, 2))

    assert_vector_similar(field.Sample((3, 4)), (1, 2))
    assert_vector_similar(
        field.Sampler()(pyCubbyFlow.Vector2D(5, 6)), (1, 2)
    )


def test_constant_vector_field3():
    field = pyCubbyFlow.ConstantVectorField3(pyCubbyFlow.Vector3D(1, 2, 3))

    assert_vector_similar(field.Sample((4, 5, 6)), (1, 2, 3))
    assert_vector_similar(
        field.Sampler()(pyCubbyFlow.Vector3D(7, 8, 9)), (1, 2, 3)
    )


def test_custom_scalar_field2():
    field = pyCubbyFlow.CustomScalarField2(
        lambda x: x.x + 2 * x.y,
        lambda x: pyCubbyFlow.Vector2D(1, 2),
        lambda x: 4,
    )

    assert field.Sample((2, 3)) == 8
    assert_vector_similar(field.Gradient((2, 3)), (1, 2))
    assert field.Laplacian((2, 3)) == 4
    assert field.Sampler()(pyCubbyFlow.Vector2D(2, 3)) == 8


def test_custom_scalar_field3():
    field = pyCubbyFlow.CustomScalarField3(
        lambda x: x.x + 2 * x.y + 3 * x.z,
        lambda x: pyCubbyFlow.Vector3D(1, 2, 3),
        lambda x: 6,
    )

    assert field.Sample((2, 3, 4)) == 20
    assert_vector_similar(field.Gradient((2, 3, 4)), (1, 2, 3))
    assert field.Laplacian((2, 3, 4)) == 6
    assert field.Sampler()(pyCubbyFlow.Vector3D(2, 3, 4)) == 20


def test_custom_vector_field2():
    field = pyCubbyFlow.CustomVectorField2(
        lambda x: pyCubbyFlow.Vector2D(x.x, -x.y),
        lambda x: 0,
        lambda x: 0,
    )

    assert_vector_similar(field.Sample((2, 3)), (2, -3))
    assert field.Divergence((2, 3)) == 0
    assert field.Curl((2, 3)) == 0
    assert_vector_similar(
        field.Sampler()(pyCubbyFlow.Vector2D(2, 3)), (2, -3)
    )


def test_custom_vector_field3():
    field = pyCubbyFlow.CustomVectorField3(
        lambda x: pyCubbyFlow.Vector3D(x.x, x.y, x.z),
        lambda x: 3,
        lambda x: pyCubbyFlow.Vector3D(1, 2, 3),
    )

    assert_vector_similar(field.Sample((2, 3, 4)), (2, 3, 4))
    assert field.Divergence((2, 3, 4)) == 3
    assert_vector_similar(field.Curl((2, 3, 4)), (1, 2, 3))
    assert_vector_similar(
        field.Sampler()(pyCubbyFlow.Vector3D(2, 3, 4)), (2, 3, 4)
    )
