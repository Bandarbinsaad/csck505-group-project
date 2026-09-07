"""Generate worlds/maze_world_complex.wbt from maze_complex.py.

Run with: uv run python generate_complex_world.py

The world's geometry is derived entirely from maze_complex.py (matrix,
cell size, start/finish), so the two never drift apart. Re-run this after
editing the maze. Style matches the hand-built worlds/maze_world.wbt
(ENU, 32 ms, e-puck with an InertialUnit and a 180-ray lidar).
"""
from pathlib import Path

import maze_complex as m

WALL_HEIGHT = 0.1
WALL_Z = 0.05
CELL = m.CELL_SIZE
FLOOR = max(m.ROW_COUNT, m.COLUMN_COUNT) * CELL
BOUNDARY_OFFSET = FLOOR / 2.0 + 0.01
BOUNDARY_LONG = FLOOR + 0.04
CAMERA_Z = round(FLOOR * 4.41, 2)

GREY = (0.55, 0.55, 0.58)
BLOCK = (0.3, 0.36, 0.46)
FLOOR_COLOUR = (0.85, 0.85, 0.85)
START_COLOUR = (0.1, 0.7, 0.2)
FINISH_COLOUR = (0.85, 0.12, 0.12)


def solidBox(name, translation, size, colour):
    """Return a Solid Box node string with a matching boundingObject."""
    tx, ty, tz = translation
    sx, sy, sz = size
    return (
        "Solid {\n"
        "  translation %g %g %g\n"
        "  children [\n"
        "    Shape {\n"
        "      appearance Appearance {\n"
        "        material Material {\n"
        "          diffuseColor %g %g %g\n"
        "        }\n"
        "      }\n"
        "      geometry Box {\n"
        "        size %g %g %g\n"
        "      }\n"
        "    }\n"
        "  ]\n"
        '  name "%s"\n'
        "  boundingObject Box {\n"
        "    size %g %g %g\n"
        "  }\n"
        "}\n"
        % (tx, ty, tz, colour[0], colour[1], colour[2],
           sx, sy, sz, name, sx, sy, sz)
    )


def marker(name, cell, colour):
    """Return a thin coloured floor marker Solid at a grid cell."""
    x, y = m.cellToWorld(*cell)
    return (
        "Solid {\n"
        "  translation %g %g 0.001\n"
        "  children [\n"
        "    Shape {\n"
        "      appearance Appearance {\n"
        "        material Material {\n"
        "          diffuseColor %g %g %g\n"
        "        }\n"
        "      }\n"
        "      geometry Box {\n"
        "        size 0.2 0.2 0.002\n"
        "      }\n"
        "    }\n"
        "  ]\n"
        '  name "%s"\n'
        "}\n" % (x, y, colour[0], colour[1], colour[2], name)
    )


def build():
    """Assemble and return the full .wbt text."""
    parts = []
    parts.append("#VRML_SIM R2025a utf8\n\n")
    parts.append(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/'
        'R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"\n\n'
    )
    parts.append(
        "WorldInfo {\n"
        "  info [\n"
        '    "CSCK505 stress-test maze: %d x %d."\n'
        '    "Geometry is defined by maze_complex.py; regenerate to match."\n'
        "  ]\n"
        '  title "CSCK505 Complex Maze"\n'
        "  basicTimeStep 32\n"
        '  coordinateSystem "ENU"\n'
        "}\n" % (m.ROW_COUNT, m.COLUMN_COUNT)
    )
    parts.append(
        "Viewpoint {\n"
        "  orientation -0.5773502691896258 0.5773502691896258 "
        "0.5773502691896258 2.0943951023931953\n"
        "  position 0 0 %g\n"
        "}\n" % CAMERA_Z
    )
    parts.append("Background {\n  skyColor [\n    0.15 0.18 0.22\n  ]\n}\n")
    parts.append(
        "DirectionalLight {\n  ambientIntensity 0.85\n  intensity 1.4\n}\n"
        "DirectionalLight {\n  ambientIntensity 0.4\n"
        "  direction 0.6 -0.6 -1\n  intensity 0.9\n}\n"
    )
    parts.append(solidBox("floor", (0, 0, -0.005),
                          (FLOOR, FLOOR, 0.01), FLOOR_COLOUR))
    parts.append(solidBox("boundaryWallNorth", (0, BOUNDARY_OFFSET, WALL_Z),
                          (BOUNDARY_LONG, 0.02, WALL_HEIGHT), GREY))
    parts.append(solidBox("boundaryWallSouth", (0, -BOUNDARY_OFFSET, WALL_Z),
                          (BOUNDARY_LONG, 0.02, WALL_HEIGHT), GREY))
    parts.append(solidBox("boundaryWallEast", (BOUNDARY_OFFSET, 0, WALL_Z),
                          (0.02, FLOOR, WALL_HEIGHT), GREY))
    parts.append(solidBox("boundaryWallWest", (-BOUNDARY_OFFSET, 0, WALL_Z),
                          (0.02, FLOOR, WALL_HEIGHT), GREY))

    for row in range(m.ROW_COUNT):
        for column in range(m.COLUMN_COUNT):
            if m.isBlocked(row, column):
                x, y = m.cellToWorld(row, column)
                parts.append(solidBox(
                    "wallBlockR%dC%d" % (row, column), (x, y, WALL_Z),
                    (CELL, CELL, WALL_HEIGHT), BLOCK))

    parts.append(marker("startMarker", m.startCell(), START_COLOUR))
    parts.append(marker("finishMarker", m.finishCell(), FINISH_COLOUR))

    startX, startY = m.cellToWorld(*m.startCell())
    parts.append(
        "E-puck {\n"
        "  translation %g %g 0\n"
        "  rotation 0 0 1 0\n"
        '  controller "micromouse"\n'
        '  controllerArgs [\n    "maze_complex"\n  ]\n'
        "  turretSlot [\n"
        "    InertialUnit {\n"
        '      name "inertial unit"\n'
        "    }\n"
        "    Lidar {\n"
        "      translation 0 0 0.02\n"
        '      name "lidar"\n'
        "      horizontalResolution 180\n"
        "      fieldOfView 6.283185\n"
        "      numberOfLayers 1\n"
        "      minRange 0.02\n"
        "      maxRange 2\n"
        "    }\n"
        "  ]\n"
        "}\n" % (startX, startY)
    )
    return "".join(parts)


def main():
    """Write the generated world next to this script."""
    outputPath = Path(__file__).resolve().parent / "worlds" / \
        "maze_world_complex.wbt"
    outputPath.write_text(build(), encoding="utf-8")
    print("wrote", outputPath)


if __name__ == "__main__":
    main()
