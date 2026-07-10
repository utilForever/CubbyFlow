import math
import pyCubbyFlow


class MyPhysicsAnimation(pyCubbyFlow.PhysicsAnimation):
    def __init__(self):
        """Initialize the test animation counters."""
        self.init_data = 0
        self.adv_data = 0
        super(MyPhysicsAnimation, self).__init__()

    def OnAdvanceTimeStep(self, timeIntervalInSeconds):
        self.adv_data += 1

    def GetNumberOfSubTimeSteps(self, timeIntervalInSeconds):
        return int(math.ceil(timeIntervalInSeconds / 0.02))

    def OnInitialize(self):
        self.init_data = 1


def test_inheritance():
    anim = MyPhysicsAnimation()

    anim.isUsingFixedSubTimeSteps = False
    f = pyCubbyFlow.Frame(index=3, timeIntervalInSeconds=0.1)
    anim.Update(f)
    assert anim.init_data == 1
    assert anim.adv_data == 20
