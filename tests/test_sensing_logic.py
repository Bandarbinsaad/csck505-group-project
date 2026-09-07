from end_of_module_assignment.maze.sensing_logic import (
    lidarToSides,
    proximityToSides,
)
from end_of_module_assignment.maze.types import Sides


def testProximityWallWhenAtOrAboveThreshold():
    # front pair (ps0, ps7) high => wall; right pair (ps1, ps2) high => wall
    values = [520, 60, 60, 60, 60, 60, 60, 500]
    result = proximityToSides(values, wallThreshold=200)
    assert result == Sides(front=True, left=False, right=False, back=False)


def testProximityUsesMaxOfEachPair():
    # only ps2 (right pair) is high; its pair-mate ps1 is low
    values = [60, 60, 480, 60, 60, 60, 60, 60]
    result = proximityToSides(values, wallThreshold=200)
    assert result == Sides(front=False, left=False, right=True, back=False)


def testLidarRayZeroIsRearRayNinetyIsFront():
    # 180 rays, 2 deg each; near return only straight ahead (ray 90)
    ranges = [1.0] * 180
    ranges[90] = 0.03
    result = lidarToSides(ranges, wallThresholdInMetres=0.05)
    assert result == Sides(front=True, left=False, right=False, back=False)


def testLidarRearReturnMarksBack():
    ranges = [1.0] * 180
    ranges[0] = 0.03  # ray 0 = rear
    result = lidarToSides(ranges, wallThresholdInMetres=0.05)
    assert result.back is True
    assert result.front is False
