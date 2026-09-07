"""Webots e-puck driver: IMU-closed-loop turns, encoder-measured distance.

The world's e-puck has wheel motors, wheel position sensors and an
InertialUnit (added for reliable heading). Turns rotate until the measured
yaw change reaches the target, so they are immune to wheel-radius error and
slip. Straight moves hold the entry heading with a proportional correction,
which removes the lateral drift that open-loop odometry accumulated. Cell
and heading are tracked internally from the known start pose.
"""
from __future__ import annotations

import math

from controller import Robot

from end_of_module_assignment.maze.types import Cell, Heading

WHEEL_RADIUS_IN_METRES = 0.02          # e-puck proto wheel cylinder radius
MAX_WHEEL_SPEED_IN_RADIANS_PER_SECOND = 6.28
CRUISE_FRACTION = 0.5
TURN_FRACTION = 0.3                 # coarse spin speed
TURN_FINE_FRACTION = 0.05          # slow approach near the target angle
TURN_SLOW_ZONE_IN_DEGREES = 12.0   # switch to fine speed within this of target
TURN_TOLERANCE_IN_DEGREES = 0.5    # stop within this of the target angle
HEADING_HOLD_GAIN_PER_DEGREE = 0.02    # wheel-speed fraction per degree error
MAX_HEADING_CORRECTION = 0.3
REVERSE_SETTLE_IN_RADIANS = 0.05
MAX_MANOEUVRE_STEPS = 400          # safety cap so a jam cannot hang a run
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LEFT_ENCODER_NAME = "left wheel sensor"
RIGHT_ENCODER_NAME = "right wheel sensor"
INERTIAL_UNIT_NAME = "inertial unit"


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
        self._startHeading = startHeading
        self._referenceYaw = 0.0  # IMU yaw when facing startHeading
        self._samplingPeriodInMs = samplingPeriodInMs
        self._isFrontBlocked = None  # optional () -> bool front bump probe

    def initialiseDevices(self) -> None:
        """Retrieve motors, encoders and the IMU, then take one step."""
        self._leftMotor = self.getDevice(LEFT_MOTOR_NAME)
        self._rightMotor = self.getDevice(RIGHT_MOTOR_NAME)
        for motor in (self._leftMotor, self._rightMotor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        self._leftEncoder = self.getDevice(LEFT_ENCODER_NAME)
        self._rightEncoder = self.getDevice(RIGHT_ENCODER_NAME)
        self._leftEncoder.enable(self._samplingPeriodInMs)
        self._rightEncoder.enable(self._samplingPeriodInMs)
        self._inertialUnit = self.getDevice(INERTIAL_UNIT_NAME)
        self._inertialUnit.enable(self._samplingPeriodInMs)
        self.step(self._samplingPeriodInMs)  # first readings become valid
        self._referenceYaw = self._yawInDegrees()  # yaw of startHeading

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

    def attachFrontProbe(self, isFrontBlockedFn) -> None:
        """Attach a front-bump test used by tryMoveForward (IR probing).

        @param isFrontBlockedFn a callable returning True when an obstacle
            is close ahead.
        """
        self._isFrontBlocked = isFrontBlockedFn

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

    def _yawInDegrees(self) -> float:
        """Return the robot's yaw from the IMU, in degrees.

        @return the yaw angle in degrees.
        """
        return math.degrees(self._inertialUnit.getRollPitchYaw()[2])

    @staticmethod
    def _shortestDeltaInDegrees(first: float, second: float) -> float:
        """Return the signed smallest difference first - second.

        @param first a bearing in degrees.
        @param second a bearing in degrees.
        @return the difference wrapped to [-180, 180].
        """
        return (first - second + 180.0) % 360.0 - 180.0

    def _absoluteYawFor(self, heading: Heading) -> float:
        """Return the IMU yaw (degrees) at which the robot faces a heading.

        Cardinals are 90 degrees apart from the reference yaw captured at
        startup; turning left (anticlockwise) increases yaw in ENU.

        @param heading the heading to face.
        @return the absolute target yaw in degrees.
        """
        anticlockwiseSteps = 0
        current = self._startHeading
        while current != heading and anticlockwiseSteps < 4:
            current = current.turnLeft()
            anticlockwiseSteps += 1
        return self._referenceYaw + 90.0 * anticlockwiseSteps

    def turnTo(self, targetHeading: Heading) -> None:
        """Rotate to the absolute yaw of a heading, correcting any drift.

        Closed-loop on the IMU and referenced to an absolute cardinal, so
        turn errors do not accumulate across the run.

        @param targetHeading the heading to adopt.
        """
        targetYaw = self._absoluteYawFor(targetHeading)
        guard = 0
        while self.step(self._samplingPeriodInMs) != -1:
            error = self._shortestDeltaInDegrees(
                targetYaw, self._yawInDegrees()
            )
            if abs(error) <= TURN_TOLERANCE_IN_DEGREES:
                break
            guard += 1
            if guard >= MAX_MANOEUVRE_STEPS:
                break
            speed = (
                TURN_FRACTION
                if abs(error) > TURN_SLOW_ZONE_IN_DEGREES
                else TURN_FINE_FRACTION
            )
            # error > 0 means we must increase yaw: turn anticlockwise (left).
            if error > 0:
                self._setWheelSpeeds(-speed, speed)
            else:
                self._setWheelSpeeds(speed, -speed)
        self._setWheelSpeeds(0.0, 0.0)
        self.step(self._samplingPeriodInMs)
        self._heading = targetHeading

    def _driveOneCell(self, isBlockedFn):
        """Drive forward one cell, holding heading, optionally bump-aborting.

        @param isBlockedFn optional callable; if it returns True the drive
            aborts before completing (used for IR probing).
        @return a tuple (startLeft, startRight, blocked) of the encoder
            values before the drive and whether it aborted.
        """
        targetRotationInRadians = (
            self._cellDistanceInMetres / WHEEL_RADIUS_IN_METRES
        )
        startLeft = self._leftEncoder.getValue()
        startRight = self._rightEncoder.getValue()
        headingYaw = self._yawInDegrees()
        blocked = False
        guard = 0
        while self.step(self._samplingPeriodInMs) != -1:
            if isBlockedFn is not None and isBlockedFn():
                blocked = True
                break
            leftDelta = abs(self._leftEncoder.getValue() - startLeft)
            rightDelta = abs(self._rightEncoder.getValue() - startRight)
            if (leftDelta + rightDelta) / 2.0 >= targetRotationInRadians:
                break
            guard += 1
            if guard >= MAX_MANOEUVRE_STEPS:
                break
            error = self._shortestDeltaInDegrees(
                self._yawInDegrees(), headingYaw
            )
            correction = max(
                -MAX_HEADING_CORRECTION,
                min(MAX_HEADING_CORRECTION,
                    HEADING_HOLD_GAIN_PER_DEGREE * error),
            )
            self._setWheelSpeeds(
                CRUISE_FRACTION + correction, CRUISE_FRACTION - correction
            )
        self._setWheelSpeeds(0.0, 0.0)
        self.step(self._samplingPeriodInMs)
        return startLeft, startRight, blocked

    def _advanceCell(self) -> None:
        """Step the tracked cell one place along the current heading."""
        rowDelta, columnDelta = self._heading.offset
        self._cell = (self._cell[0] + rowDelta, self._cell[1] + columnDelta)

    def moveForward(self) -> None:
        """Drive forward one cell and update the tracked cell."""
        self._driveOneCell(None)
        self._advanceCell()

    def _reverseToStart(self, startLeft: float, startRight: float) -> None:
        """Reverse until the wheels return near their starting angles.

        @param startLeft the left encoder value before the aborted move.
        @param startRight the right encoder value before the aborted move.
        """
        self._setWheelSpeeds(-CRUISE_FRACTION, -CRUISE_FRACTION)
        guard = 0
        while self.step(self._samplingPeriodInMs) != -1:
            leftDelta = abs(self._leftEncoder.getValue() - startLeft)
            rightDelta = abs(self._rightEncoder.getValue() - startRight)
            if (leftDelta + rightDelta) / 2.0 <= REVERSE_SETTLE_IN_RADIANS:
                break
            guard += 1
            if guard >= MAX_MANOEUVRE_STEPS:
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
        startLeft, startRight, blocked = self._driveOneCell(
            self._isFrontBlocked
        )
        if blocked:
            self._reverseToStart(startLeft, startRight)
            return False
        self._advanceCell()
        return True
