"""In-memory doubles for exercising explorer + mapper without Webots."""
from __future__ import annotations

from .types import Cell, Heading, Sides


class FakeMaze:
    """An occupancy grid where True marks a wall; off-grid is wall too."""

    def __init__(self, occupancy: list[list[bool]]) -> None:
        """Wrap an occupancy grid.

        @param occupancy rows of booleans, True where a wall block sits.
        """
        self.occupancy = occupancy
        self.rowCount = len(occupancy)
        self.columnCount = len(occupancy[0]) if occupancy else 0

    def isWall(self, cell: Cell) -> bool:
        """Report whether a cell holds a wall or lies off-grid.

        @param cell the (row, column) to test.
        @return True if blocked or outside the grid.
        """
        row, column = cell
        if not (0 <= row < self.rowCount and 0 <= column < self.columnCount):
            return True
        return self.occupancy[row][column]


class FakeRobot:
    """A RobotDriver double: turns are instant, forward moves one open cell."""

    def __init__(
        self, maze: FakeMaze, startCell: Cell,
        startHeading: Heading = Heading.N,
    ) -> None:
        """Place the robot in a maze.

        @param maze the maze to move within.
        @param startCell the initial (row, column).
        @param startHeading the initial heading.
        """
        self._maze = maze
        self._cell = startCell
        self._heading = startHeading

    @property
    def heading(self) -> Heading:
        return self._heading

    @property
    def cell(self) -> Cell:
        return self._cell

    def turnTo(self, targetHeading: Heading) -> None:
        """Face the given heading immediately.

        @param targetHeading the heading to adopt.
        """
        self._heading = targetHeading

    def moveForward(self) -> None:
        """Advance one cell if the cell ahead is open."""
        rowDelta, columnDelta = self._heading.offset
        target = (self._cell[0] + rowDelta, self._cell[1] + columnDelta)
        if not self._maze.isWall(target):
            self._cell = target


class FakeSensor:
    """A WallSensor double reading the maze around the robot."""

    def __init__(self, maze: FakeMaze, robot: FakeRobot) -> None:
        """Bind the sensor to a maze and robot.

        @param maze the maze to read.
        @param robot the robot whose pose frames the reading.
        """
        self._maze = maze
        self._robot = robot

    def read(self) -> Sides:
        """Return walls around the robot in its heading frame.

        @return the egocentric wall reading.
        """
        cell = self._robot.cell
        heading = self._robot.heading

        def wall(sideHeading: Heading) -> bool:
            rowDelta, columnDelta = sideHeading.offset
            return self._maze.isWall(
                (cell[0] + rowDelta, cell[1] + columnDelta)
            )

        return Sides(
            front=wall(heading),
            right=wall(heading.turnRight()),
            left=wall(heading.turnLeft()),
            back=wall(heading.opposite()),
        )
