"""Render a map.json dump to a PNG with the travelled path in red."""

from __future__ import annotations

import json
import sys

_COLOUR_FOR_SYMBOL = {
    "1": (0.15, 0.15, 0.15),
    "0": (0.95, 0.95, 0.95),
    "?": (0.60, 0.60, 0.85),
}
_PATH_COLOUR = (0.9, 0.1, 0.1)


def matrixToRgb(
    matrix: list[list[str]], path: list
) -> list[list[tuple[float, float, float]]]:
    """Colour each cell: red on the path, else by its symbol.

    @param matrix rows of '?', '0', '1'.
    @param path list of [row, column] cells the robot travelled.
    @return rows of (r, g, b) tuples in 0..1.
    """
    pathCells = {tuple(cell) for cell in path}
    rgb: list[list[tuple[float, float, float]]] = []
    for rowIndex, row in enumerate(matrix):
        rgbRow = [
            _PATH_COLOUR
            if (rowIndex, columnIndex) in pathCells
            else _COLOUR_FOR_SYMBOL.get(symbol, _COLOUR_FOR_SYMBOL["?"])
            for columnIndex, symbol in enumerate(row)
        ]
        rgb.append(rgbRow)
    return rgb


def render(jsonPath: str, pngPath: str) -> None:
    """Read a map.json and write a labelled PNG.

    @param jsonPath path to the map.json dump.
    @param pngPath path for the output PNG.
    """
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot

    with open(jsonPath, encoding="utf-8") as handle:
        data = json.load(handle)

    matrix = data["matrix"]
    rgb = matrixToRgb(matrix, data.get("path", []))
    rowCount = len(matrix)
    columnCount = len(matrix[0]) if matrix else 0

    figure, axes = pyplot.subplots(
        figsize=(columnCount * 0.5 + 1, rowCount * 0.5 + 1)
    )
    axes.imshow(rgb)
    axes.set_xticks(range(columnCount))
    axes.set_yticks(range(rowCount))
    sensorName = data.get("sensorName", "")
    timeInSeconds = data.get("timeInSeconds", 0.0)
    axes.set_title(f"Map ({sensorName}) - time {timeInSeconds:.2f} s")
    figure.savefig(pngPath, dpi=150, bbox_inches="tight")
    pyplot.close(figure)


def main(argv: list[str] | None = None) -> None:
    """CLI entry: render-map <map.json> [out.png].

    @param argv optional argument list (defaults to sys.argv[1:]).
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    jsonPath = argv[0] if argv else "map.json"
    pngPath = argv[1] if len(argv) > 1 else jsonPath.replace(".json", ".png")
    render(jsonPath, pngPath)


if __name__ == "__main__":
    main()
