"""Depth-first exploration with backtracking, building the map as it goes.

At each cell the robot senses all four sides and, among open neighbours not
yet visited, chooses one by the fixed priority straight->right->left->back
and moves there (pushing it on the path stack). When no unvisited open
neighbour remains it backtracks one cell along the stack. This always
terminates and reaches the goal if it is reachable - unlike a purely greedy
rule, it cannot circle forever.
"""
from __future__ import annotations

from dataclasses import dataclass

from .interfaces import RobotDriver, WallSensor
from .mapper import Map
from .types import Action, Cell, Heading, Sides, headingBetween

_PRIORITY = (Action.FORWARD, Action.RIGHT, Action.LEFT, Action.BACK)

_SIDE_FIELD = {
    Action.FORWARD: "front",
    Action.RIGHT: "right",
    Action.LEFT: "left",
    Action.BACK: "back",
}


@dataclass
class ExploreResult:
    """Outcome of an exploration run."""

    reachedGoal: bool
    stepCount: int
    cellsVisited: int


def _openUnvisitedHeading(
    walls: Sides,
    heading: Heading,
    cell: Cell,
    visitedCells: set[Cell],
) -> Heading | None:
    """Return the heading to the best open, unvisited neighbour, or None.

    Candidates are considered in the priority order straight, right, left,
    back (relative to the current heading).

    @param walls the egocentric wall reading at the cell.
    @param heading the robot's current heading.
    @param cell the robot's current (row, column).
    @param visitedCells cells already visited.
    @return the absolute heading to move, or None if none qualifies.
    """
    for action in _PRIORITY:
        if getattr(walls, _SIDE_FIELD[action]):
            continue
        candidate = action.toHeading(heading)
        rowDelta, columnDelta = candidate.offset
        neighbour = (cell[0] + rowDelta, cell[1] + columnDelta)
        if neighbour not in visitedCells:
            return candidate
    return None


def explore(
    robot: RobotDriver,
    sensor: WallSensor,
    grid: Map,
    goalCell: Cell,
    maxStepCount: int = 1000,
) -> ExploreResult:
    """Run depth-first exploration with backtracking, building the map.

    @param robot the driver that moves and reports pose.
    @param sensor the wall sensor.
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @param maxStepCount safety cap on cells stepped.
    @return the ExploreResult for the run.
    """
    cell = robot.cell
    visitedCells: set[Cell] = {cell}
    pathStack: list[Cell] = [cell]
    grid.markPath(cell)
    grid.markWalls(cell, robot.heading, sensor.read())

    stepCount = 0
    while cell != goalCell and stepCount < maxStepCount:
        walls = sensor.read()
        grid.markWalls(cell, robot.heading, walls)
        forward = _openUnvisitedHeading(
            walls, robot.heading, cell, visitedCells
        )
        if forward is not None:
            robot.turnTo(forward)
            robot.moveForward()
            cell = robot.cell
            visitedCells.add(cell)
            pathStack.append(cell)
        else:
            pathStack.pop()
            if not pathStack:
                break  # fully explored, goal unreachable
            robot.turnTo(headingBetween(cell, pathStack[-1]))
            robot.moveForward()
            cell = robot.cell
        grid.markPath(cell)
        stepCount += 1

    grid.markWalls(cell, robot.heading, sensor.read())
    return ExploreResult(
        reachedGoal=(cell == goalCell),
        stepCount=stepCount,
        cellsVisited=len(visitedCells),
    )
