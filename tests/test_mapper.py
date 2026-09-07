import json

from end_of_module_assignment.maze.mapper import (
    FREE,
    OBSTACLE,
    UNKNOWN,
    Map,
)
from end_of_module_assignment.maze.types import Heading, Sides


def testNewMapIsAllUnknown():
    grid = Map(3, 4)
    assert grid.toMatrix() == [[UNKNOWN] * 4 for _ in range(3)]


def testMarkWallsMarksNeighboursAndCurrent():
    grid = Map(3, 3)
    grid.markWalls(
        (1, 1), Heading.N,
        Sides(front=True, left=False, right=False, back=False),
    )
    matrix = grid.toMatrix()
    assert matrix[1][1] == FREE     # current cell
    assert matrix[0][1] == OBSTACLE  # north neighbour is a wall
    assert matrix[1][2] == FREE     # east (right) open
    assert matrix[1][0] == FREE     # west (left) open
    assert matrix[2][1] == FREE     # south (back) open


def testMarkWallsIgnoresOutOfBounds():
    grid = Map(2, 2)
    grid.markWalls(
        (0, 0), Heading.N, Sides(False, False, False, False)
    )
    assert grid.toMatrix()[0][0] == FREE


def testMarkPathRecordsOrderWithoutDuplicates():
    grid = Map(2, 2)
    grid.markPath((0, 0))
    grid.markPath((0, 1))
    grid.markPath((0, 1))
    assert grid.path == [(0, 0), (0, 1)]
    assert grid.toMatrix()[0][1] == FREE


def testDumpWritesJsonAndText(tmp_path):
    grid = Map(2, 2)
    grid.markPath((0, 0))
    grid.markObstacle((1, 1))
    prefix = str(tmp_path / "map")
    grid.dump(prefix, sensorName="proximity", timeInSeconds=1.5, stepCount=3)
    data = json.loads((tmp_path / "map.json").read_text())
    assert data["sensorName"] == "proximity"
    assert data["timeInSeconds"] == 1.5
    assert data["matrix"][1][1] == OBSTACLE
    assert data["path"] == [[0, 0]]
    assert (tmp_path / "map.txt").exists()
