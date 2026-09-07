"""Micromouse controller: reactive priority exploration + map building.

Set SENSOR_NAME to "proximity" (Experiment 1) or "lidar" (Experiment 2).
All other logic is identical between runs. The maze, start and finish come
from the group's maze.py (single source of truth). Reload the world before
each run. Thresholds are placeholders - calibrate against the smoke-test
evidence (proximity wall ~150-250; lidar wall ~0.15 m for a 0.25 m cell).
"""
from __future__ import annotations

import sys
from pathlib import Path

# maze.py lives at the repo root (two levels above this controller).
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import maze  # noqa: E402  (repo-root single source of truth)
from robot import EpuckRobot  # noqa: E402
from sensing import LidarWallSensor, ProximitySensorArray  # noqa: E402

from end_of_module_assignment.maze.explorer import explore  # noqa: E402
from end_of_module_assignment.maze.mapper import Map  # noqa: E402
from end_of_module_assignment.maze.types import Heading  # noqa: E402

SENSOR_NAME = "proximity"           # "proximity" | "lidar"
START_HEADING = Heading.E           # e-puck starts facing east
PROXIMITY_WALL_THRESHOLD = 200.0    # calibrate
LIDAR_WALL_THRESHOLD_IN_METRES = 0.15  # calibrate
MAX_STEP_COUNT = 200


def buildSensor(robot, samplingPeriodInMs):
    """Construct the wall sensor named by SENSOR_NAME.

    @param robot the EpuckRobot owning the devices.
    @param samplingPeriodInMs sampling period in milliseconds.
    @return a WallSensor.
    """
    if SENSOR_NAME == "lidar":
        return LidarWallSensor(
            robot, samplingPeriodInMs, LIDAR_WALL_THRESHOLD_IN_METRES
        )
    return ProximitySensorArray(
        robot, samplingPeriodInMs, PROXIMITY_WALL_THRESHOLD
    )


def main():
    """Run one exploration, print the map, and dump the map files."""
    startCell = maze.startCell()
    goalCell = maze.finishCell()

    robot = EpuckRobot(
        cellDistanceInMetres=maze.CELL_SIZE,
        startCell=startCell,
        startHeading=START_HEADING,
    )
    robot.initialiseDevices()
    samplingPeriodInMs = int(robot.getBasicTimeStep())

    sensor = buildSensor(robot, samplingPeriodInMs)
    grid = Map(maze.ROW_COUNT, maze.COLUMN_COUNT)

    startTimeInSeconds = robot.getTime()
    result = explore(
        robot, sensor, grid, goalCell, maxStepCount=MAX_STEP_COUNT
    )
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
