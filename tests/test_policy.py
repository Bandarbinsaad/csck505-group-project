from mazebot.maze.policy import chooseMove
from mazebot.maze.types import Action, Sides

NONE = Sides(False, False, False, False)


def walls(front=False, left=False, right=False, back=False):
    return Sides(front, left, right, back)


def testPrefersStraightWhenOpenAndUnvisited():
    assert chooseMove(walls=NONE, visitedSides=NONE) is Action.FORWARD


def testPriorityIsRightThenLeftThenBack():
    assert chooseMove(walls(front=True), NONE) is Action.RIGHT
    assert chooseMove(walls(front=True, right=True), NONE) is Action.LEFT
    assert (
        chooseMove(walls(front=True, right=True, left=True), NONE)
        is Action.BACK
    )


def testPrefersUnvisitedOverPriority():
    visited = Sides(front=True, left=False, right=False, back=True)
    assert chooseMove(NONE, visited) is Action.RIGHT


def testFallsBackToPriorityWhenAllOpenSidesVisited():
    someWalls = walls(left=True, back=True)
    allVisited = Sides(front=True, left=True, right=True, back=True)
    assert chooseMove(someWalls, allVisited) is Action.FORWARD


def testReturnsNoneWhenFullyEnclosed():
    enclosed = walls(front=True, left=True, right=True, back=True)
    assert chooseMove(enclosed, NONE) is None
