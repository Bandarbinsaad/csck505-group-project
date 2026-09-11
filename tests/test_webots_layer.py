import importlib
import sys
from pathlib import Path

CONTROLLER_DIR = (
    Path(__file__).resolve().parents[1] / "controllers" / "micromouse"
)
sys.path.insert(0, str(CONTROLLER_DIR))

from mazebot.maze.interfaces import (
    RobotDriver,
    WallSensor,
)
from mazebot.maze.types import Heading, Sides


def testProximityArrayReadsSides(fakeController):
    sensing = importlib.reload(importlib.import_module("sensing"))
    robot = fakeController.Robot()
    robot.getDevice("ps0").value = 520  # front pair high
    robot.getDevice("ps7").value = 60
    robot.getDevice("ps2").value = 480  # right pair high
    array = sensing.ProximitySensorArray(
        robot, samplingPeriodInMs=32, wallThreshold=200
    )
    result = array.read()
    assert isinstance(array, WallSensor)
    assert result == Sides(front=True, left=False, right=True, back=False)


def testLidarReadsFrontAtRayNinety(fakeController):
    sensing = importlib.reload(importlib.import_module("sensing"))
    robot = fakeController.Robot()
    ranges = [1.0] * 180
    ranges[90] = 0.03
    robot.getDevice("lidar").rangeImage = ranges
    sensor = sensing.LidarWallSensor(
        robot, samplingPeriodInMs=32, wallThresholdInMetres=0.05
    )
    result = sensor.read()
    assert isinstance(sensor, WallSensor)
    assert result == Sides(front=True, left=False, right=False, back=False)


def testEpuckRobotConstructsAndReportsPose(fakeController):
    robotModule = importlib.reload(importlib.import_module("robot"))
    epuck = robotModule.EpuckRobot(
        cellDistanceInMetres=0.25, startCell=(0, 0), startHeading=Heading.E
    )
    epuck.initialiseDevices()
    assert isinstance(epuck, RobotDriver)
    assert epuck.cell == (0, 0)
    assert epuck.heading is Heading.E
