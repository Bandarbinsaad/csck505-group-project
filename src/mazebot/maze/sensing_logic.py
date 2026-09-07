"""Pure reductions from raw sensor readings to four wall booleans."""
from __future__ import annotations

from .types import Sides


def proximityToSides(
    proximityValues: list[float], wallThreshold: float
) -> Sides:
    """Reduce eight e-puck IR proximity readings to four wall booleans.

    IR proximity rises as an obstacle nears, so a side is walled when its
    reading is at or above the threshold. Pairs follow the world hand-off:
    front (ps0, ps7), right (ps1, ps2), back (ps3, ps4), left (ps5, ps6).

    @param proximityValues the eight ps0..ps7 readings, in order.
    @param wallThreshold the proximity value at or above which a wall is
        deemed present.
    @return the egocentric wall reading.
    """
    front = max(proximityValues[0], proximityValues[7])
    right = max(proximityValues[1], proximityValues[2])
    back = max(proximityValues[3], proximityValues[4])
    left = max(proximityValues[5], proximityValues[6])
    return Sides(
        front=front >= wallThreshold,
        left=left >= wallThreshold,
        right=right >= wallThreshold,
        back=back >= wallThreshold,
    )


def _angularDistanceInDegrees(
    firstDegrees: float, secondDegrees: float
) -> float:
    """Return the smallest angle between two bearings.

    @param firstDegrees a bearing in degrees.
    @param secondDegrees a bearing in degrees.
    @return the absolute separation in [0, 180].
    """
    delta = abs(firstDegrees - secondDegrees) % 360.0
    return min(delta, 360.0 - delta)


def lidarToSides(
    rangesInMetres: list[float],
    wallThresholdInMetres: float,
    startBearingInDegrees: float = 180.0,
    sectorHalfInDegrees: float = 15.0,
) -> Sides:
    """Reduce a 360-degree LiDAR scan to four wall booleans.

    Beam i points at (startBearingInDegrees + i*360/n) mod 360 in the robot
    frame, where front is 0, right 90, back 180, left 270. The world's LiDAR
    puts ray 0 to the rear, so the default start bearing is 180 degrees. A
    side is walled when the minimum range within +/- sectorHalfInDegrees of
    that cardinal is at or below the threshold.

    @param rangesInMetres one range per beam.
    @param wallThresholdInMetres range at or below which a wall is present.
    @param startBearingInDegrees bearing of beam index 0.
    @param sectorHalfInDegrees half-width of each cardinal window.
    @return the egocentric wall reading.
    """
    beamCount = len(rangesInMetres)
    degreesPerBeam = 360.0 / beamCount if beamCount else 0.0

    def blocked(cardinalDegrees: float) -> bool:
        window = [
            rangesInMetres[i]
            for i in range(beamCount)
            if _angularDistanceInDegrees(
                (startBearingInDegrees + i * degreesPerBeam) % 360.0,
                cardinalDegrees,
            )
            <= sectorHalfInDegrees
        ]
        return bool(window) and min(window) <= wallThresholdInMetres

    return Sides(
        front=blocked(0.0),
        right=blocked(90.0),
        back=blocked(180.0),
        left=blocked(270.0),
    )
