"""Structural interfaces the pure logic depends on (typing.Protocol)."""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from .types import Cell, Heading, Sides


@runtime_checkable
class WallSensor(Protocol):
    """Reports walls on the robot's four egocentric sides."""

    def read(self) -> Sides:
        """Return the current wall reading.

        @return a Sides where True means a wall on that side.
        """
        ...


@runtime_checkable
class RobotDriver(Protocol):
    """Moves the robot one grid cell at a time and tracks its pose."""

    @property
    def heading(self) -> Heading: ...

    @property
    def cell(self) -> Cell: ...

    def turnTo(self, targetHeading: Heading) -> None:
        """Rotate in place to face the given heading."""
        ...

    def moveForward(self) -> None:
        """Drive forward exactly one grid cell."""
        ...


@runtime_checkable
class ProbeRobotDriver(Protocol):
    """A driver that discovers walls by attempting moves (IR probing).

    Used where the sensor cannot see a wall a full cell away, so the robot
    tries to move and reports whether the cell ahead was open.
    """

    @property
    def heading(self) -> Heading: ...

    @property
    def cell(self) -> Cell: ...

    def turnTo(self, targetHeading: Heading) -> None:
        """Rotate in place to face the given heading."""
        ...

    def tryMoveForward(self) -> bool:
        """Attempt to advance one cell.

        @return True if the robot advanced, False if a wall blocked it
            (the robot stays on its current cell).
        """
        ...
