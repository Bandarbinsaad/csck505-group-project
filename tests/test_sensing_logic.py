from mazebot.maze.sensing_logic import (
    lidarToSides,
    proximityToSides,
)
from mazebot.maze.types import Sides


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


def testProximityBackPairHigh():
    # back pair (ps3, ps4) high => wall
    values = [60, 60, 60, 480, 60, 60, 60, 60]
    result = proximityToSides(values, wallThreshold=200)
    assert result == Sides(front=False, left=False, right=False,
                           back=True)


def testProximityLeftPairHigh():
    # left pair (ps5, ps6) high => wall
    values = [60, 60, 60, 60, 60, 60, 470, 60]
    result = proximityToSides(values, wallThreshold=200)
    assert result == Sides(front=False, left=True, right=False,
                           back=False)


def testLidarRightCardinal():
    # ray 135 is the 90° right cardinal (180 rays, 2° each)
    ranges = [1.0] * 180
    ranges[135] = 0.03
    result = lidarToSides(ranges, wallThresholdInMetres=0.05)
    assert result.right is True
    assert result.front is False


def testLidarLeftCardinal():
    # ray 45 is the 270° left cardinal (180 rays, 2° each)
    ranges = [1.0] * 180
    ranges[45] = 0.03
    result = lidarToSides(ranges, wallThresholdInMetres=0.05)
    assert result.left is True
    assert result.front is False
