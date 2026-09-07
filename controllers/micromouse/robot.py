"""Webots e-puck driver using wheel-encoder odometry (no GPS/IMU).

The world's e-puck exposes wheel motors and wheel position sensors but no
GPS or inertial unit, so distance and turns are measured from encoder
deltas. Heading and cell are tracked internally: each forward move steps the
cell along the current heading; each turn updates the heading enum.

Geometry constants are standard e-puck values - verify against the proto.
"""
from __future__ import annotations

import math

from controller import Robot

from end_of_module_assignment.maze.types import Cell, Heading

WHEEL_RADIUS_IN_METRES = 0.0205        # verify against E-puck.proto
AXLE_LENGTH_IN_METRES = 0.052          # verify against E-puck.proto
MAX_WHEEL_SPEED_IN_RADIANS_PER_SECOND = 6.28
CRUISE_FRACTION = 0.5
TURN_FRACTION = 0.3
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LEFT_ENCODER_NAME = "left wheel sensor"
RIGHT_ENCODER_NAME = "right wheel sensor"


class EpuckRobot(Robot):
    """Drives the e-puck one cell / one 90-degree turn at a time."""

    def __init__(
        self,
        cellDistanceInMetres: float,
        startCell: Cell,
        startHeading: Heading = Heading.E,
        samplingPeriodInMs: int = 32,
    ) -> None:
        """Configure pose tracking (call initialiseDevices next).

        @param cellDistanceInMetres world distance between cell centres.
        @param startCell the robot's start (row, column).
        @param startHeading the robot's start heading.
        @param samplingPeriodInMs control period in milliseconds.
        """
        super().__init__()
        self._cellDistanceInMetres = cellDistanceInMetres
        self._cell = startCell
        self._heading = startHeading
        self._samplingPeriodInMs = samplingPeriodInMs
        self._isFrontBlocked = None  # optional () -> bool front bump probe

    def initialiseDevices(self) -> None:
        """Retrieve motors and encoders and take one settling step."""
        self._leftMotor = self.getDevice(LEFT_MOTOR_NAME)
        self._rightMotor = self.getDevice(RIGHT_MOTOR_NAME)
        for motor in (self._leftMotor, self._rightMotor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        self._leftEncoder = self.getDevice(LEFT_ENCODER_NAME)
        self._rightEncoder = self.getDevice(RIGHT_ENCODER_NAME)
        self._leftEncoder.enable(self._samplingPeriodInMs)
        self._rightEncoder.enable(self._samplingPeriodInMs)
        self.step(self._samplingPeriodInMs)  # first reading becomes valid

    @property
    def heading(self) -> Heading:
        """Return the robot's current tracked heading.

        @return the current heading.
        """
        return self._heading

    @property
    def cell(self) -> Cell:
        """Return the robot's current tracked cell.

        @return the current (row, column).
        """
        return self._cell

    def _setWheelSpeeds(
        self, leftFraction: float, rightFraction: float
    ) -> None:
        """Command both wheels as fractions of max speed.

        @param leftFraction left wheel speed fraction in [-1, 1].
        @param rightFraction right wheel speed fraction in [-1, 1].
        """
        maxSpeed = MAX_WHEEL_SPEED_IN_RADIANS_PER_SECOND
        self._leftMotor.setVelocity(leftFraction * maxSpeed)
        self._rightMotor.setVelocity(rightFraction * maxSpeed)

    def _driveWheelRotation(
        self, targetRotationInRadians: float, clockwise: bool | None
    ) -> None:
        """Drive until the reference wheel turns the target amount.

        When clockwise is None the robot drives straight (both wheels
        forward) and progress is the average wheel rotation. Otherwise it
        spins in place and progress is the left wheel's absolute rotation.

        @param targetRotationInRadians wheel rotation to accumulate.
        @param clockwise True to spin right, False left, None to go
            straight.
        """
        startLeft = self._leftEncoder.getValue()
        startRight = self._rightEncoder.getValue()
        if clockwise is None:
            self._setWheelSpeeds(CRUISE_FRACTION, CRUISE_FRACTION)
        elif clockwise:
            self._setWheelSpeeds(TURN_FRACTION, -TURN_FRACTION)
        else:
            self._setWheelSpeeds(-TURN_FRACTION, TURN_FRACTION)

        while self.step(self._samplingPeriodInMs) != -1:
            leftDelta = abs(self._leftEncoder.getValue() - startLeft)
            rightDelta = abs(self._rightEncoder.getValue() - startRight)
            progress = (
                (leftDelta + rightDelta) / 2.0
                if clockwise is None
                else leftDelta
            )
            if progress >= targetRotationInRadians:
                break
        self._setWheelSpeeds(0.0, 0.0)
        self.step(self._samplingPeriodInMs)

    def _rotate(self, angleInDegrees: float) -> None:
        """Spin in place by a signed angle (positive is clockwise).

        @param angleInDegrees the turn, positive clockwise (right).
        """
        arcInMetres = math.radians(abs(angleInDegrees)) * (
            AXLE_LENGTH_IN_METRES / 2.0
        )
        targetRotationInRadians = arcInMetres / WHEEL_RADIUS_IN_METRES
        self._driveWheelRotation(
            targetRotationInRadians, clockwise=angleInDegrees > 0
        )

    def turnTo(self, targetHeading: Heading) -> None:
        """Rotate in place to face a heading and update the pose.

        @param targetHeading the heading to adopt.
        """
        if targetHeading == self._heading.turnRight():
            self._rotate(90.0)
        elif targetHeading == self._heading.turnLeft():
            self._rotate(-90.0)
        elif targetHeading != self._heading:
            self._rotate(180.0)
        self._heading = targetHeading

    def moveForward(self) -> None:
        """Drive forward one cell and update the tracked cell."""
        targetRotationInRadians = (
            self._cellDistanceInMetres / WHEEL_RADIUS_IN_METRES
        )
        self._driveWheelRotation(targetRotationInRadians, clockwise=None)
        rowDelta, columnDelta = self._heading.offset
        self._cell = (self._cell[0] + rowDelta, self._cell[1] + columnDelta)

    def attachFrontProbe(self, isFrontBlockedFn) -> None:
        """Attach a front-bump test used by tryMoveForward (IR probing).

        @param isFrontBlockedFn a callable returning True when an obstacle
            is close ahead.
        """
        self._isFrontBlocked = isFrontBlockedFn

    def _reverseToStart(self, startLeft: float, startRight: float) -> None:
        """Reverse until the wheels return near their starting angles.

        @param startLeft the left encoder value before the aborted move.
        @param startRight the right encoder value before the aborted move.
        """
        self._setWheelSpeeds(-CRUISE_FRACTION, -CRUISE_FRACTION)
        while self.step(self._samplingPeriodInMs) != -1:
            leftDelta = abs(self._leftEncoder.getValue() - startLeft)
            rightDelta = abs(self._rightEncoder.getValue() - startRight)
            if (leftDelta + rightDelta) / 2.0 <= 0.05:
                break
        self._setWheelSpeeds(0.0, 0.0)
        self.step(self._samplingPeriodInMs)

    def tryMoveForward(self) -> bool:
        """Drive forward one cell, aborting if the front bump fires.

        Used for short-range IR probing: if an obstacle is detected close
        ahead before a full cell is covered, stop, reverse to the starting
        pose, and report the direction as blocked (the cell is unchanged).

        @return True if a full cell was covered, False if blocked.
        """
        targetRotationInRadians = (
            self._cellDistanceInMetres / WHEEL_RADIUS_IN_METRES
        )
        startLeft = self._leftEncoder.getValue()
        startRight = self._rightEncoder.getValue()
        self._setWheelSpeeds(CRUISE_FRACTION, CRUISE_FRACTION)

        blocked = False
        while self.step(self._samplingPeriodInMs) != -1:
            if self._isFrontBlocked is not None and self._isFrontBlocked():
                blocked = True
                break
            leftDelta = abs(self._leftEncoder.getValue() - startLeft)
            rightDelta = abs(self._rightEncoder.getValue() - startRight)
            if (leftDelta + rightDelta) / 2.0 >= targetRotationInRadians:
                break
        self._setWheelSpeeds(0.0, 0.0)
        self.step(self._samplingPeriodInMs)

        if blocked:
            self._reverseToStart(startLeft, startRight)
            return False
        rowDelta, columnDelta = self._heading.offset
        self._cell = (self._cell[0] + rowDelta, self._cell[1] + columnDelta)
        return True
