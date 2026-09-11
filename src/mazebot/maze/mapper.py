"""Incremental binary occupancy map built from sensor readings."""

from __future__ import annotations

import json

from .types import Cell, Heading, Sides

UNKNOWN = "?"
FREE = "0"
OBSTACLE = "1"


class Map:
    """A row-by-column grid of '?', '0', '1' plus the travelled path."""

    def __init__(self, rowCount: int, columnCount: int) -> None:
        """Create an all-unknown map.

        @param rowCount number of grid rows.
        @param columnCount number of grid columns.
        """
        self.rowCount = rowCount
        self.columnCount = columnCount
        self._grid = [[UNKNOWN] * columnCount for _ in range(rowCount)]
        self.path: list[Cell] = []

    def _inBounds(self, cell: Cell) -> bool:
        """Return whether a cell lies inside the grid.

        @param cell the (row, column) to test.
        @return True if the cell is on the grid.
        """
        row, column = cell
        return 0 <= row < self.rowCount and 0 <= column < self.columnCount

    def _set(self, cell: Cell, state: str) -> None:
        """Set a cell's state if it is on the grid.

        @param cell the (row, column) to set.
        @param state one of UNKNOWN, FREE, OBSTACLE.
        """
        if self._inBounds(cell):
            self._grid[cell[0]][cell[1]] = state

    def markFree(self, cell: Cell) -> None:
        """Mark a cell as sensed free.

        @param cell the (row, column) to mark.
        """
        self._set(cell, FREE)

    def markObstacle(self, cell: Cell) -> None:
        """Mark a cell as a sensed obstacle.

        @param cell the (row, column) to mark.
        """
        self._set(cell, OBSTACLE)

    def markWalls(self, cell: Cell, heading: Heading, walls: Sides) -> None:
        """Record walls sensed from a cell into its four neighbours.

        The current cell is marked free (the robot stands on it). Each
        egocentric side marks its absolute neighbour obstacle or free.

        @param cell the robot's current (row, column).
        @param heading the robot's current heading.
        @param walls the egocentric wall reading.
        """
        self.markFree(cell)
        egocentric = {
            heading: walls.front,
            heading.turnRight(): walls.right,
            heading.turnLeft(): walls.left,
            heading.opposite(): walls.back,
        }
        for sideHeading, isWall in egocentric.items():
            rowDelta, columnDelta = sideHeading.offset
            neighbour = (cell[0] + rowDelta, cell[1] + columnDelta)
            if isWall:
                self.markObstacle(neighbour)
            else:
                self.markFree(neighbour)

    def markPath(self, cell: Cell) -> None:
        """Record that the robot occupied a cell.

        @param cell the (row, column) the robot occupies.
        """
        self.markFree(cell)
        if not self.path or self.path[-1] != cell:
            self.path.append(cell)

    def toMatrix(self) -> list[list[str]]:
        """Return a deep copy of the symbol grid.

        @return a list of rows of '?', '0' or '1'.
        """
        return [row[:] for row in self._grid]

    def stats(self) -> dict:
        """Return counts of each cell state and the path length.

        @return a dict with free, obstacle, unknown, pathLength.
        """
        flat = [cell for row in self._grid for cell in row]
        return {
            "free": flat.count(FREE),
            "obstacle": flat.count(OBSTACLE),
            "unknown": flat.count(UNKNOWN),
            "pathLength": len(self.path),
        }

    def toDict(
        self,
        sensorName: str = "",
        timeInSeconds: float = 0.0,
        stepCount: int = 0,
    ) -> dict:
        """Serialise the map and run metadata to a plain dict.

        @param sensorName the sensor used for this run.
        @param timeInSeconds the competition time.
        @param stepCount the number of cells stepped.
        @return a JSON-ready dict.
        """
        return {
            "sensorName": sensorName,
            "timeInSeconds": timeInSeconds,
            "stepCount": stepCount,
            "rowCount": self.rowCount,
            "columnCount": self.columnCount,
            "matrix": self.toMatrix(),
            "path": [list(cell) for cell in self.path],
            "stats": self.stats(),
        }

    def dump(
        self,
        pathPrefix: str,
        sensorName: str = "",
        timeInSeconds: float = 0.0,
        stepCount: int = 0,
    ) -> None:
        """Write <prefix>.json and <prefix>.txt to disk.

        @param pathPrefix path stem for the two output files.
        @param sensorName the sensor used for this run.
        @param timeInSeconds the competition time.
        @param stepCount the number of cells stepped.
        """
        data = self.toDict(sensorName, timeInSeconds, stepCount)
        with open(pathPrefix + ".json", "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
        with open(pathPrefix + ".txt", "w", encoding="utf-8") as handle:
            handle.writelines(" ".join(row) + "\n" for row in self.toMatrix())
