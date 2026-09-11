"""Maze definition for the CSCK505 end-of-module assignment.

Single source of truth for the maze layout; the wall blocks in
``worlds/maze_world.wbt`` correspond to the ``#`` cells here, so if either
is edited the other must be kept in step.

Grid convention: zero-based ``(row, column)`` from the top-left; row
increases southwards, column increases eastwards.

Symbols: ``S`` start, ``F`` finish, ``.`` free, ``#`` wall block.

Naming note: the CSCK505 standard requires UPPER_CASE constants, so the
matrix is named ``MAZE`` (character-for-character identical to the brief).
"""

MAZE = [
    "S...#",
    "###.#",
    "....#",
    ".##..",
    "...#F",
]

CELL_SIZE = 0.25

START_SYMBOL = "S"
FINISH_SYMBOL = "F"

ROW_COUNT = len(MAZE)
COLUMN_COUNT = len(MAZE[0])


def symbolAt(row, column):
    """Return the matrix symbol at a grid cell.

    :param row: zero-based row index.
    :param column: zero-based column index.
    :return: the single-character symbol stored at that cell.
    """
    return MAZE[row][column]


def findSymbol(symbol):
    """Return the (row, column) of the first occurrence of a symbol.

    :param symbol: the single-character symbol to locate.
    :return: tuple (row, column) of zero-based grid indices.
    :raises ValueError: if the symbol does not appear in MAZE.
    """
    for row in range(ROW_COUNT):
        for column in range(COLUMN_COUNT):
            if symbolAt(row, column) == symbol:
                return row, column
    raise ValueError(f"symbol {symbol!r} not present in MAZE")


def startCell():
    """Return the (row, column) of the start cell.

    :return: tuple (row, column) of zero-based grid indices.
    """
    return findSymbol(START_SYMBOL)


def finishCell():
    """Return the (row, column) of the finish cell.

    :return: tuple (row, column) of zero-based grid indices.
    """
    return findSymbol(FINISH_SYMBOL)
