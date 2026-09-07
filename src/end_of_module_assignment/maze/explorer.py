"""The exploration loop: sense, map, decide, move, until goal or cap."""
from __future__ import annotations

from dataclasses import dataclass

from .interfaces import RobotDriver, WallSensor
from .mapper import Map
from .policy import chooseMove
from .types import Cell, Heading, Sides


@dataclass
class ExploreResult:
    """Outcome of an exploration run."""

    reachedGoal: bool
    stepCount: int
    cellsVisited: int


def _visitedSides(
    cell: Cell, heading: Heading, visitedCells: set[Cell]
) -> Sides:
    """Return which egocentric sides lead to already-visited cells.

    @param cell the robot's current (row, column).
    @param heading the robot's current heading.
    @param visitedCells the set of cells already visited.
    @return a Sides where True means that side's neighbour was visited.
    """
    def seen(sideHeading: Heading) -> bool:
        rowDelta, columnDelta = sideHeading.offset
        return (cell[0] + rowDelta, cell[1] + columnDelta) in visitedCells

    return Sides(
        front=seen(heading),
        right=seen(heading.turnRight()),
        left=seen(heading.turnLeft()),
        back=seen(heading.opposite()),
    )


def explore(
    robot: RobotDriver,
    sensor: WallSensor,
    grid: Map,
    goalCell: Cell,
    maxStepCount: int = 1000,
) -> ExploreResult:
    """Run the reactive priority behaviour, building the map.

    @param robot the driver that moves and reports pose.
    @param sensor the wall sensor.
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @param maxStepCount safety cap on cells stepped.
    @return the ExploreResult for the run.
    """
    cell = robot.cell
    visitedCells: set[Cell] = {cell}
    grid.markPath(cell)
    walls = sensor.read()
    grid.markWalls(cell, robot.heading, walls)

    stepCount = 0
    while cell != goalCell and stepCount < maxStepCount:
        action = chooseMove(
            walls, _visitedSides(cell, robot.heading, visitedCells)
        )
        if action is None:
            break
        robot.turnTo(action.toHeading(robot.heading))
        robot.moveForward()
        cell = robot.cell
        visitedCells.add(cell)
        grid.markPath(cell)
        walls = sensor.read()
        grid.markWalls(cell, robot.heading, walls)
        stepCount += 1

    return ExploreResult(
        reachedGoal=(cell == goalCell),
        stepCount=stepCount,
        cellsVisited=len(visitedCells),
    )
