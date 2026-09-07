"""Micromouse controller: reactive priority exploration + map building.

Set SENSOR_NAME to "proximity" (Experiment 1) or "lidar" (Experiment 2).
All other logic is identical between runs. The maze, start and finish come
from the group's maze.py (single source of truth). Reload the world before
each run.

Calibration note: the e-puck accumulates lateral odometry drift, so a wall
one cell away is read at 0.12-0.21 m rather than the nominal 0.125 m. The
lidar threshold sits in the measured gap between "wall present" (<= ~0.21 m)
and "cell open" (>= ~0.29 m). Proximity (IR) cannot see a wall a full cell
away at all (see the report / smoke-test evidence); Experiment 1 is included
to demonstrate that sensor limitation.
"""
from __future__ import annotations

import sys
from pathlib import Path

# The repo root (two levels above this controller) holds maze.py; its src/
# holds the end_of_module_assignment package. Adding both to sys.path lets
# Webots run this controller under any interpreter it launches, without
# needing the uv venv on the Python command.
_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "src"))
sys.path.insert(0, str(_REPO_ROOT))

import maze  # noqa: E402  (repo-root single source of truth)
from robot import EpuckRobot  # noqa: E402
from sensing import LidarWallSensor, ProximitySensorArray  # noqa: E402

from end_of_module_assignment.maze.explorer import explore  # noqa: E402
from end_of_module_assignment.maze.mapper import Map  # noqa: E402
from end_of_module_assignment.maze.types import Heading  # noqa: E402

SENSOR_NAME = "lidar"  # "proximity" | "lidar"
START_HEADING = Heading.E           # e-puck starts facing east
PROXIMITY_WALL_THRESHOLD = 200.0    # calibrate (IR rises as a wall nears)
LIDAR_WALL_THRESHOLD_IN_METRES = 0.22  # calibrated to the drift-widened gap
MAX_STEP_COUNT = 200
WARM_UP_STEP_COUNT = 3              # sensors need a step after enable


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

    # Sensor data is only valid after at least one simulation step following
    # enable(); the lidar's getRangeImage() crashes if read before then.
    for _ in range(WARM_UP_STEP_COUNT):
        robot.step(samplingPeriodInMs)

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
