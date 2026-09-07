"""Webots wall sensors reducing device readings to a Sides value.

The reduction logic lives in the pure sensing_logic module; these classes
only wire Webots devices to it.
"""
from __future__ import annotations

from end_of_module_assignment.maze.sensing_logic import (
    lidarToSides,
    proximityToSides,
)
from end_of_module_assignment.maze.types import Sides

PROXIMITY_SENSOR_NAMES = (
    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7",
)
LIDAR_DEVICE_NAME = "lidar"


class ProximitySensorArray:
    """The eight e-puck IR proximity sensors, read as four sides."""

    def __init__(
        self, robot, samplingPeriodInMs: int, wallThreshold: float
    ) -> None:
        """Retrieve and enable ps0..ps7.

        @param robot the Webots Robot owning the devices.
        @param samplingPeriodInMs sampling period in milliseconds.
        @param wallThreshold proximity value at or above which a wall is
            deemed present.
        """
        self._wallThreshold = wallThreshold
        self._sensors = []
        for sensorName in PROXIMITY_SENSOR_NAMES:
            sensor = robot.getDevice(sensorName)
            sensor.enable(samplingPeriodInMs)
            self._sensors.append(sensor)

    def read(self) -> Sides:
        """Return the current wall reading.

        @return the egocentric wall reading.
        """
        values = [sensor.getValue() for sensor in self._sensors]
        return proximityToSides(values, self._wallThreshold)


class LidarWallSensor:
    """The 360-degree LiDAR reduced to four cardinal wall booleans."""

    def __init__(
        self,
        robot,
        samplingPeriodInMs: int,
        wallThresholdInMetres: float,
        sectorHalfInDegrees: float = 15.0,
    ) -> None:
        """Retrieve and enable the LiDAR.

        @param robot the Webots Robot owning the device.
        @param samplingPeriodInMs sampling period in milliseconds.
        @param wallThresholdInMetres range at or below which a wall is
            present.
        @param sectorHalfInDegrees half-width of each cardinal window.
        """
        self._wallThresholdInMetres = wallThresholdInMetres
        self._sectorHalfInDegrees = sectorHalfInDegrees
        self._lidar = robot.getDevice(LIDAR_DEVICE_NAME)
        self._lidar.enable(samplingPeriodInMs)

    def read(self) -> Sides:
        """Return the current wall reading.

        @return the egocentric wall reading.
        """
        ranges = list(self._lidar.getRangeImage())
        return lidarToSides(
            ranges,
            self._wallThresholdInMetres,
            sectorHalfInDegrees=self._sectorHalfInDegrees,
        )
