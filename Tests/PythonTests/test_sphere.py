import pyCubbyFlow


def test_init():
    a = pyCubbyFlow.Sphere3()
    assert not a.isNormalFlipped
    assert a.center == (0, 0, 0)
    assert a.radius == 1.0
