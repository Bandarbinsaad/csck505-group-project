"""Micromouse controller: reactive exploration + map building.

Two experiments, selected by SENSOR_NAME:
  "lidar"     - Experiment 2: a 360 deg range scan senses all four sides
                from the cell centre, driving the priority-rule explorer.
  "proximity" - Experiment 1: short-range IR cannot see a wall a full cell
                away, so the robot probes - it tries to move and treats a
                front bump as a wall (the wall-following / probe-and-map
                explorer). Only bumped walls are mapped, so its map is
                sparser than the lidar's - the intended sensor contrast.

The maze, start and finish come from the group's maze.py (single source of
truth). Reload the world before each run.

Calibration note: the e-puck accumulates lateral odometry drift, so a wall
one cell away is read at 0.12-0.21 m rather than the nominal 0.125 m; the
lidar threshold sits in the measured gap to "cell open" (>= ~0.29 m).
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

import importlib  # noqa: E402

# The maze module is chosen by the robot's Webots controllerArgs (e.g.
# "maze_complex"); it defaults to "maze". Both are single sources of truth.
_MAZE_MODULE_NAME = sys.argv[1] if len(sys.argv) > 1 else "maze"
maze = importlib.import_module(_MAZE_MODULE_NAME)

from robot import EpuckRobot  # noqa: E402
from sensing import LidarWallSensor, ProximitySensorArray  # noqa: E402

from end_of_module_assignment.maze.explorer import explore  # noqa: E402
from end_of_module_assignment.maze.mapper import Map  # noqa: E402
from end_of_module_assignment.maze.prober import exploreByProbing  # noqa: E402
from end_of_module_assignment.maze.types import Heading  # noqa: E402

SENSOR_NAME = "lidar"  # "proximity" | "lidar"
START_HEADING = Heading.E              # e-puck starts facing east
PROXIMITY_WALL_THRESHOLD = 200.0       # IR rises as a wall nears
PROXIMITY_PROBE_CONTACT = 200.0        # front IR bump level - CALIBRATE
LIDAR_WALL_THRESHOLD_IN_METRES = 0.22  # calibrated to the drift-widened gap
MAX_STEP_COUNT = 200
WARM_UP_STEP_COUNT = 3                 # sensors need a step after enable


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

    NOTE: the IR probe is pose-fragile. Each blocked probe noses toward a
    wall and reverses, and turns add error, so the estimated pose drifts;
    PROXIMITY_PROBE_CONTACT trades false bumps (too low) against driving
    through walls (too high). Reliable IR mapping needs per-cell
    re-centring / heading correction - tune this interactively in Webots.
    This demonstrates the core limitation being compared against the lidar.

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
