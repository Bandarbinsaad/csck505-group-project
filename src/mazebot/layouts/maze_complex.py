"""Complex 8 x 8 maze for stress-testing the robots.

Same API and conventions as maze: a longer serpentine route with a dead-end
trap, so the reactive robot must turn many times and backtrack. The wall
blocks must stay in step with worlds/maze_world_complex.wbt.
"""

MAZE = [
    "S.....#.",
    "#####.#.",
    "...#..#.",
    ".#.##.#.",
    ".#....#.",
    ".#.####.",
    ".#.....#",
    "...###.F",
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
