"""Core value types shared across the maze logic.

Grid convention: Cell is (row, column); row increases south, column
increases east. This matches the group's maze.py and the ENU world.
"""
from __future__ import annotations

from enum import Enum
from typing import NamedTuple

Cell = tuple[int, int]

_HEADING_ORDER: list["Heading"] = []


class Heading(Enum):
    """A cardinal direction with its (rowDelta, columnDelta) step."""

    N = (-1, 0)
    E = (0, 1)
    S = (1, 0)
    W = (0, -1)

    @property
    def offset(self) -> Cell:
        """Return the (rowDelta, columnDelta) for one step this way.

        @return tuple of integer row and column deltas.
        """
        return self.value

    def turnRight(self) -> "Heading":
        """Return the heading 90 degrees clockwise of this one.

        @return the clockwise neighbour heading.
        """
        return _HEADING_ORDER[(_HEADING_ORDER.index(self) + 1) % 4]

    def turnLeft(self) -> "Heading":
        """Return the heading 90 degrees anticlockwise of this one.

        @return the anticlockwise neighbour heading.
        """
        return _HEADING_ORDER[(_HEADING_ORDER.index(self) - 1) % 4]

    def opposite(self) -> "Heading":
        """Return the heading facing the reverse of this one.

        @return the opposite heading.
        """
        return _HEADING_ORDER[(_HEADING_ORDER.index(self) + 2) % 4]


_HEADING_ORDER.extend([Heading.N, Heading.E, Heading.S, Heading.W])


class Action(Enum):
    """An egocentric move relative to the robot's current heading."""

    FORWARD = "forward"
    RIGHT = "right"
    LEFT = "left"
    BACK = "back"

    def toHeading(self, currentHeading: Heading) -> Heading:
        """Resolve this action into an absolute heading.

        @param currentHeading the robot's present heading.
        @return the heading the robot faces after taking this action.
        """
        if self is Action.FORWARD:
            return currentHeading
        if self is Action.RIGHT:
            return currentHeading.turnRight()
        if self is Action.LEFT:
            return currentHeading.turnLeft()
        return currentHeading.opposite()


class Sides(NamedTuple):
    """Four egocentric booleans (front, left, right, back)."""

    front: bool
    left: bool
    right: bool
    back: bool
