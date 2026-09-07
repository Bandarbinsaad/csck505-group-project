from end_of_module_assignment.maze.types import Action, Heading, Sides


def testHeadingTurnsAreCyclic():
    assert Heading.N.turnRight() is Heading.E
    assert Heading.E.turnRight() is Heading.S
    assert Heading.W.turnRight() is Heading.N
    assert Heading.N.turnLeft() is Heading.W
    assert Heading.N.opposite() is Heading.S


def testHeadingOffsetsMatchGridConvention():
    assert Heading.N.offset == (-1, 0)
    assert Heading.S.offset == (1, 0)
    assert Heading.E.offset == (0, 1)
    assert Heading.W.offset == (0, -1)


def testActionToHeading():
    assert Action.FORWARD.toHeading(Heading.E) is Heading.E
    assert Action.RIGHT.toHeading(Heading.E) is Heading.S
    assert Action.LEFT.toHeading(Heading.E) is Heading.N
    assert Action.BACK.toHeading(Heading.E) is Heading.W


def testSidesIsFourNamedBooleans():
    sides = Sides(front=True, left=False, right=False, back=True)
    assert (sides.front, sides.left, sides.right, sides.back) == (
        True, False, False, True
    )
