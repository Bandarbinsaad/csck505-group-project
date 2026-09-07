"""Probe-and-map exploration for short-range sensors (e.g. e-puck IR).

Where a sensor cannot detect a wall a full cell away, the robot cannot
sense-then-decide. Instead it tries to move in priority order
(straight, right, left, back), preferring unvisited neighbours: a completed
move means the cell was open; a blocked move (front bump) marks that
neighbour as a wall. Only bumped walls are mapped, so the resulting map is
sparser than a range sensor's - a deliberate, instructive contrast.
"""
from __future__ import annotations

from .explorer import ExploreResult
from .interfaces import ProbeRobotDriver
from .mapper import Map
from .types import Action, Cell, Heading

_PRIORITY = (Action.FORWARD, Action.RIGHT, Action.LEFT, Action.BACK)


def exploreByProbing(
    robot: ProbeRobotDriver,
    grid: Map,
    goalCell: Cell,
    maxStepCount: int = 1000,
) -> ExploreResult:
    """Explore by attempting moves, mapping walls only where blocked.

    @param robot the probing driver (turnTo + tryMoveForward).
    @param grid the map to populate.
    @param goalCell the finish (row, column).
    @param maxStepCount safety cap on completed moves.
    @return the ExploreResult for the run.
    """
    cell = robot.cell
    visited: set[Cell] = {cell}
    knownWalls: set[Cell] = set()
    grid.markPath(cell)

    stepCount = 0
    while cell != goalCell and stepCount < maxStepCount:
        startHeading = robot.heading
        headings = [action.toHeading(startHeading) for action in _PRIORITY]

        def neighbour(heading: Heading) -> Cell:
            rowDelta, columnDelta = heading.offset
            return (cell[0] + rowDelta, cell[1] + columnDelta)

        ordered = [h for h in headings if neighbour(h) not in visited]
        ordered += [h for h in headings if neighbour(h) in visited]

        moved = False
        for heading in ordered:
            target = neighbour(heading)
            if target in knownWalls:
                continue
            robot.turnTo(heading)
            if robot.tryMoveForward():
                cell = robot.cell
                visited.add(cell)
                grid.markPath(cell)
                moved = True
                break
            knownWalls.add(target)
            grid.markObstacle(target)

        if not moved:
            break
        stepCount += 1

    return ExploreResult(
        reachedGoal=(cell == goalCell),
        stepCount=stepCount,
        cellsVisited=len(visited),
    )
