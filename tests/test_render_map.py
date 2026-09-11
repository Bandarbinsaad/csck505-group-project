import json

from mazebot.render_map import matrixToRgb, render

RED = (0.9, 0.1, 0.1)


def testPathCellsAreRed():
    matrix = [["0", "0"], ["1", "?"]]
    rgb = matrixToRgb(matrix, path=[[0, 0], [0, 1]])
    assert rgb[0][0] == RED
    assert rgb[0][1] == RED
    assert rgb[1][0] != RED
    assert rgb[1][1] != RED


def testRenderWritesPng(tmp_path):
    data = {
        "matrix": [["0", "1"], ["?", "0"]],
        "path": [[0, 0], [1, 1]],
        "sensorName": "proximity",
        "timeInSeconds": 2.0,
    }
    jsonPath = tmp_path / "map.json"
    jsonPath.write_text(json.dumps(data))
    pngPath = tmp_path / "map.png"
    render(str(jsonPath), str(pngPath))
    assert pngPath.exists() and pngPath.stat().st_size > 0
