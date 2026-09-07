from mazebot.maze.explorer import ExploreResult
from mazebot.maze.interfaces import ProbeRobotDriver
from mazebot.maze.mapper import FREE, OBSTACLE, UNKNOWN, Map
from mazebot.maze.offline_sim import FakeMaze, FakeRobot
from mazebot.maze.prober import exploreByProbing
from mazebot.maze.types import Heading

# The group's 5x5 maze (True = wall block). Mirrors maze.py MAZE.
GROUP_MAZE = [
    [False, False, False, False, True],
    [True, True, True, False, True],
    [False, False, False, False, True],
    [False, True, True, False, False],
    [False, False, False, True, False],
]


def runProbing(occupancy, startCell, goalCell, startHeading, maxStepCount):
    maze = FakeMaze(occupancy)
    robot = FakeRobot(maze, startCell, startHeading)
    grid = Map(maze.rowCount, maze.columnCount)
    result = exploreByProbing(robot, grid, goalCell, maxStepCount)
    return result, grid


def testFakeRobotSatisfiesProbeProtocol():
    robot = FakeRobot(FakeMaze(GROUP_MAZE), (0, 0))
    assert isinstance(robot, ProbeRobotDriver)


def testTryMoveForwardReportsOpenAndBlocked():
    maze = FakeMaze(GROUP_MAZE)
    robot = FakeRobot(maze, (0, 3), Heading.E)  # east of (0,3) is (0,4) wall
    assert robot.tryMoveForward() is False  # blocked, stays
    assert robot.cell == (0, 3)
    robot.turnTo(Heading.S)  # south is (1,3) open
    assert robot.tryMoveForward() is True
    assert robot.cell == (1, 3)


def testSolvesGroupMazeByProbing():
    result, grid = runProbing(GROUP_MAZE, (0, 0), (4, 4), Heading.E, 200)
    assert isinstance(result, ExploreResult)
    assert result.reachedGoal is True
    matrix = grid.toMatrix()
    # only bumped walls are mapped; these three are on the tried route
    assert matrix[0][4] == OBSTACLE
    assert matrix[3][2] == OBSTACLE
    assert matrix[4][3] == OBSTACLE
    # the goal cell is on the travelled path
    assert matrix[4][4] == FREE
    # a wall never adjacent to the route stays unknown (sparser than lidar)
    assert matrix[1][0] == UNKNOWN


def testProbingUnreachableGoalTerminates():
    walled = [
        [False, True, False],
        [True, True, False],
        [False, False, False],
    ]
    result, _ = runProbing(walled, (2, 2), (0, 0), Heading.N, 100)
    assert result.reachedGoal is False
    assert result.stepCount <= 100
