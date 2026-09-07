"""Micromouse controller: reactive exploration and map building.

SENSOR_NAME selects the experiment: "lidar" senses all four sides from the
cell centre; "proximity" uses short-range IR, which cannot see a wall a full
cell away and so discovers walls by probing (bumping) into them. The maze is
chosen by the robot's Webots controllerArgs and defaults to "maze".
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

_MAZE_NAME = sys.argv[1] if len(sys.argv) > 1 else "maze"
maze = importlib.import_module(
    "end_of_module_assignment.layouts." + _MAZE_NAME
)

from robot import EpuckRobot  # noqa: E402
from sensing import LidarWallSensor, ProximitySensorArray  # noqa: E402

from end_of_module_assignment.maze.explorer import explore  # noqa: E402
from end_of_module_assignment.maze.mapper import Map  # noqa: E402
from end_of_module_assignment.maze.prober import exploreByProbing  # noqa: E402
from end_of_module_assignment.maze.types import Heading  # noqa: E402

SENSOR_NAME = "lidar"  # "proximity" | "lidar"
START_HEADING = Heading.E
PROXIMITY_WALL_THRESHOLD = 200.0
PROXIMITY_PROBE_CONTACT = 200.0
LIDAR_WALL_THRESHOLD_IN_METRES = 0.22
MAX_STEP_COUNT = 200
WARM_UP_STEP_COUNT = 3


def warmUp(robot, samplingPeriodInMs):
    """Step a few times so newly enabled sensors return valid data.

    @param robot the EpuckRobot.
    @param samplingPeriodInMs sampling period in milliseconds.
    """
    for _ in range(WARM_UP_STEP_COUNT):
        robot.step(samplingPeriodInMs)


def runLidar(robot, samplingPeriodInMs, grid, goalCell):
    """Experiment 2: explore with the lidar wall sensor.

    @param robot the EpuckRobot.
    @param samplingPeriodInMs sampling period in milliseconds.
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @return the ExploreResult.
    """
    sensor = LidarWallSensor(
        robot, samplingPeriodInMs, LIDAR_WALL_THRESHOLD_IN_METRES
    )
    warmUp(robot, samplingPeriodInMs)
    return explore(robot, sensor, grid, goalCell, maxStepCount=MAX_STEP_COUNT)


def runProximity(robot, samplingPeriodInMs, grid, goalCell):
    """Experiment 1: explore by IR probing (front-bump wall following).

    @param robot the EpuckRobot.
    @param samplingPeriodInMs sampling period in milliseconds.
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @return the ExploreResult.
    """
    proximity = ProximitySensorArray(
        robot, samplingPeriodInMs, PROXIMITY_WALL_THRESHOLD
    )
    warmUp(robot, samplingPeriodInMs)
    robot.attachFrontProbe(
        lambda: proximity.frontValue() >= PROXIMITY_PROBE_CONTACT
    )
    return exploreByProbing(
        robot, grid, goalCell, maxStepCount=MAX_STEP_COUNT
    )


def main():
    """Run one exploration, print the map, and dump the map files."""
    robot = EpuckRobot(
        cellDistanceInMetres=maze.CELL_SIZE,
        startCell=maze.startCell(),
        startHeading=START_HEADING,
    )
    robot.initialiseDevices()
    samplingPeriodInMs = int(robot.getBasicTimeStep())
    goalCell = maze.finishCell()
    grid = Map(maze.ROW_COUNT, maze.COLUMN_COUNT)

    startTimeInSeconds = robot.getTime()
    if SENSOR_NAME == "proximity":
        result = runProximity(robot, samplingPeriodInMs, grid, goalCell)
    else:
        result = runLidar(robot, samplingPeriodInMs, grid, goalCell)
    elapsedInSeconds = robot.getTime() - startTimeInSeconds

    grid.dump(
        "map_" + SENSOR_NAME, sensorName=SENSOR_NAME,
        timeInSeconds=elapsedInSeconds, stepCount=result.stepCount,
    )
    for row in grid.toMatrix():
        print(" ".join(row))
    print(
        "sensor=%s reachedGoal=%s time=%.2fs steps=%d visited=%d"
        % (
            SENSOR_NAME, result.reachedGoal, elapsedInSeconds,
            result.stepCount, result.cellsVisited,
        )
    )


if __name__ == "__main__":
    main()
