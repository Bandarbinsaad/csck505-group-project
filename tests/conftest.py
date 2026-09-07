"""A fake Webots `controller` module so the Webots layer imports off-sim."""
import sys
import types as pythonTypes

import pytest


class FakeDevice:
    """Stands in for a motor, sensor, encoder or lidar."""

    def __init__(self):
        self.value = 0.0
        self.rangeImage = []

    def enable(self, samplingPeriodInMs):
        pass

    def enablePointCloud(self):
        pass

    def setPosition(self, position):
        pass

    def setVelocity(self, velocity):
        pass

    def getValue(self):
        return self.value

    def getRangeImage(self):
        return self.rangeImage


class FakeRobot:
    """Minimal Robot: hands out FakeDevices, ends step loops at once."""

    def __init__(self):
        self.devices = {}

    def getDevice(self, name):
        return self.devices.setdefault(name, FakeDevice())

    def getBasicTimeStep(self):
        return 32.0

    def step(self, samplingPeriodInMs):
        return -1

    def getTime(self):
        return 0.0


@pytest.fixture
def fakeController(monkeypatch):
    module = pythonTypes.ModuleType("controller")
    module.Robot = FakeRobot
    module.Lidar = FakeDevice
    module.DistanceSensor = FakeDevice
    monkeypatch.setitem(sys.modules, "controller", module)
    return module
