"""The fixed-priority reactive movement rule (behaviour-based control)."""

from __future__ import annotations

from .types import Action, Sides

_PRIORITY = (Action.FORWARD, Action.RIGHT, Action.LEFT, Action.BACK)

_FIELD_FOR_ACTION = {
    Action.FORWARD: "front",
    Action.RIGHT: "right",
    Action.LEFT: "left",
    Action.BACK: "back",
}


def _sideFlag(sides: Sides, action: Action) -> bool:
    """Return the Sides flag for the side an action moves toward.

    @param sides the four egocentric booleans.
    @param action the candidate action.
    @return the boolean for that action's side.
    """
    return getattr(sides, _FIELD_FOR_ACTION[action])


def chooseMove(walls: Sides, visitedSides: Sides) -> Action | None:
    """Pick a move by priority straight, right, left, back.

    Among sides with no wall, prefer one whose neighbour is unvisited; if
    all open sides are visited, fall back to the highest-priority open side
    (the backtracking case). Return None only when every side is walled.

    @param walls True where a wall blocks that side.
    @param visitedSides True where that side's neighbour is already visited.
    @return the chosen Action, or None if enclosed.
    """
    openActions = [a for a in _PRIORITY if not _sideFlag(walls, a)]
    if not openActions:
        return None
    unvisited = [a for a in openActions if not _sideFlag(visitedSides, a)]
    return unvisited[0] if unvisited else openActions[0]
