from pytest import approx


def assert_vector_similar(a, b):
    assert tuple(a) == approx(tuple(b))


def assert_bounding_box_similar(a, b):
    assert_vector_similar(a.lowerCorner, b.lowerCorner)
    assert_vector_similar(a.upperCorner, b.upperCorner)
