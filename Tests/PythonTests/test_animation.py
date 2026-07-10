import pyCubbyFlow


class MyAnimation(pyCubbyFlow.Animation):
    def __init__(self):
        """Initialize the test animation state."""
        self.lastFrame = None
        super(MyAnimation, self).__init__()

    def OnUpdate(self, frame):
        self.lastFrame = frame


def test_inheritance():
    anim = MyAnimation()
    f = pyCubbyFlow.Frame(3, 0.02)
    anim.Update(f)
    assert anim.lastFrame.index == 3
    assert anim.lastFrame.timeIntervalInSeconds == 0.02
