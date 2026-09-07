"""Maze definition for the CSCK505 end-of-module assignment.

Single source of truth for the maze layout and for the mapping between
grid cells and Webots world coordinates. The world file
``worlds/maze_world.wbt`` is built to match these values exactly; if
either is edited, the other must be edited to match.

Grid convention
---------------
Zero-based ``(row, column)`` indexing from the top-left of the matrix.
Row increases southwards, which is decreasing world y. Column increases
eastwards, which is increasing world x.

Symbols
-------
``S`` start cell, ``F`` finish cell, ``.`` traversable floor, ``#``
blocked cell physically occupied by a solid wall block.

Naming note
-----------
The CSCK505 coding standard requires UPPER_CASE for constants, so the
matrix is named ``MAZE`` rather than ``maze`` as written in the task
brief. It is otherwise character-for-character identical.
"""

MAZE = [
    "S...#",
    "###.#",
    "....#",
    ".##..",
    "...#F",
]

CELL_SIZE = 0.25
ORIGIN = (-0.625, 0.625)

START_SYMBOL = "S"
FINISH_SYMBOL = "F"
FREE_SYMBOL = "."
BLOCKED_SYMBOL = "#"

ROW_COUNT = len(MAZE)
COLUMN_COUNT = len(MAZE[0])


def cellToWorld(row, column):
    """Return the world (x, y) centre of a grid cell, in metres.

    :param row: zero-based row index, counted from the top of MAZE.
    :param column: zero-based column index, counted from the left.
    :return: tuple (xInMetres, yInMetres) of the cell centre.
    """
    originX, originY = ORIGIN
    xInMetres = originX + (column + 0.5) * CELL_SIZE
    yInMetres = originY - (row + 0.5) * CELL_SIZE
    return xInMetres, yInMetres


def worldToCell(xInMetres, yInMetres):
    """Return the (row, column) grid cell containing a world position.

    No bounds checking is performed; a position outside the arena
    returns an index outside the matrix.

    :param xInMetres: world x coordinate in metres.
    :param yInMetres: world y coordinate in metres.
    :return: tuple (row, column) of zero-based grid indices.
    """
    originX, originY = ORIGIN
    column = int((xInMetres - originX) // CELL_SIZE)
    row = int((originY - yInMetres) // CELL_SIZE)
    return row, column


def symbolAt(row, column):
    """Return the matrix symbol at a grid cell.

    :param row: zero-based row index.
    :param column: zero-based column index.
    :return: the single-character symbol stored at that cell.
    """
    return MAZE[row][column]


def isBlocked(row, column):
    """Report whether a grid cell is occupied by a wall block.

    Cells outside the matrix are treated as blocked, because the arena
    boundary wall stands there.

    :param row: zero-based row index.
    :param column: zero-based column index.
    :return: True if the cell is a wall or outside the arena.
    """
    if row < 0 or row >= ROW_COUNT:
        return True
    if column < 0 or column >= COLUMN_COUNT:
        return True
    return symbolAt(row, column) == BLOCKED_SYMBOL


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
