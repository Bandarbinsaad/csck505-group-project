from end_of_module_assignment.maze.explorer import ExploreResult, explore
from end_of_module_assignment.maze.interfaces import RobotDriver, WallSensor
from end_of_module_assignment.maze.mapper import Map, OBSTACLE
from end_of_module_assignment.maze.offline_sim import (
    FakeMaze,
    FakeRobot,
    FakeSensor,
)
from end_of_module_assignment.maze.types import Heading

# The group's 5x5 maze (True = wall block). Mirrors maze.py MAZE.
GROUP_MAZE = [
    [False, False, False, False, True],
    [True, True, True, False, True],
    [False, False, False, False, True],
    [False, True, True, False, False],
    [False, False, False, True, False],
]


def runExploration(occupancy, startCell, goalCell, startHeading, maxStepCount):
    maze = FakeMaze(occupancy)
    robot = FakeRobot(maze, startCell, startHeading)
    sensor = FakeSensor(maze, robot)
    grid = Map(maze.rowCount, maze.columnCount)
    result = explore(robot, sensor, grid, goalCell, maxStepCount)
    return result, grid


def testFakesSatisfyProtocols():
    maze = FakeMaze(GROUP_MAZE)
    robot = FakeRobot(maze, (0, 0))
    assert isinstance(robot, RobotDriver)
    assert isinstance(FakeSensor(maze, robot), WallSensor)


def testSolvesGroupMazeWithoutLooping():
    result, grid = runExploration(
        GROUP_MAZE, (0, 0), (4, 4), Heading.E, 200
    )
    assert isinstance(result, ExploreResult)
    assert result.reachedGoal is True
    assert result.stepCount == 8       # the documented 9-cell route
    assert grid.toMatrix()[0][4] == OBSTACLE  # a sensed wall


def testUnreachableGoalTerminates():
    walled = [
        [False, True, False],
        [True, True, False],
        [False, False, False],
    ]
    result, _ = runExploration(walled, (2, 2), (0, 0), Heading.N, 100)
    assert result.reachedGoal is False
    assert result.stepCount <= 100


def testPathStartsAtStartCell():
    _, grid = runExploration(GROUP_MAZE, (0, 0), (4, 4), Heading.E, 200)
    assert grid.path[0] == (0, 0)
