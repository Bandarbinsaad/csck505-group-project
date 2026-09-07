"""Depth-first probe-and-map exploration for short-range sensors (IR).

Where a sensor cannot detect a wall a full cell away, the robot discovers
walls by trying to move. At each cell it tries unvisited directions in the
priority order straight->right->left->back; a completed move descends into
that cell, a blocked move (front bump) marks that neighbour a wall. When no
unvisited direction opens, it backtracks one cell along the path stack, so
it cannot circle forever. Only bumped walls are mapped, so the map is
sparser than a range sensor's - a deliberate, instructive contrast.
"""
from __future__ import annotations

from .explorer import ExploreResult
from .interfaces import ProbeRobotDriver
from .mapper import Map
from .types import Action, Cell, headingBetween

_PRIORITY = (Action.FORWARD, Action.RIGHT, Action.LEFT, Action.BACK)


def exploreByProbing(
    robot: ProbeRobotDriver,
    grid: Map,
    goalCell: Cell,
    maxStepCount: int = 1000,
) -> ExploreResult:
    """Explore depth-first by attempting moves, mapping bumped walls.

    @param robot the probing driver (turnTo + tryMoveForward).
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @param maxStepCount safety cap on completed moves.
    @return the ExploreResult for the run.
    """
    cell = robot.cell
    visited: set[Cell] = {cell}
    knownWalls: set[Cell] = set()
    pathStack: list[Cell] = [cell]
    grid.markPath(cell)

    stepCount = 0
    while cell != goalCell and stepCount < maxStepCount:
        startHeading = robot.heading
        moved = False
        for action in _PRIORITY:
            heading = action.toHeading(startHeading)
            rowDelta, columnDelta = heading.offset
            neighbour = (cell[0] + rowDelta, cell[1] + columnDelta)
            if neighbour in visited or neighbour in knownWalls:
                continue
            robot.turnTo(heading)
            if robot.tryMoveForward():
                cell = robot.cell
                visited.add(cell)
                pathStack.append(cell)
                grid.markPath(cell)
                moved = True
                break
            knownWalls.add(neighbour)
            grid.markObstacle(neighbour)

        if moved:
            stepCount += 1
            continue

        pathStack.pop()
        if not pathStack:
            break  # fully explored, goal unreachable
        robot.turnTo(headingBetween(cell, pathStack[-1]))
        robot.tryMoveForward()
        cell = robot.cell
        grid.markPath(cell)
        stepCount += 1

    return ExploreResult(
        reachedGoal=(cell == goalCell),
        stepCount=stepCount,
        cellsVisited=len(visited),
    )
