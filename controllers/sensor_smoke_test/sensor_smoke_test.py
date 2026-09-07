"""Temporary environment-validation controller for the CSCK505 maze.

NOT ASSESSED CODE. NOT PART OF THE FINAL SUBMISSION.

This controller exists only to prove that ``worlds/maze_world.wbt``
exposes the devices the group's navigation controller will need. It
deliberately performs no navigation, no wall following, no map
building, no finish detection and no competition timing. It drives
forward slowly for a fixed period, prints device summaries to the
Webots console, and stops.

The assessed controller is owned by Alex Senger and replaces this one.
See ``README_WORLD_HANDOFF.md`` for the integration steps.

Naming follows the CSCK505 coding standard: camelCase for variables
and functions, UPPER_CASE for constants, Pydoc docstrings, four-space
indentation and 79-character lines.
"""

from controller import Robot

LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
PROXIMITY_SENSOR_NAMES = (
    "ps0",
    "ps1",
    "ps2",
    "ps3",
    "ps4",
    "ps5",
    "ps6",
    "ps7",
)
LIDAR_DEVICE_NAME = "lidar"

SELF_RETURN_THRESHOLD_IN_METRES = 0.05
DRIVE_SPEED_IN_RADIANS_PER_SECOND = 2.0
DRIVE_DURATION_IN_SECONDS = 2.0
REPORT_INTERVAL_IN_SECONDS = 0.5


def initialiseMotors(robot):
    """Configure both wheel motors for velocity control, stopped.

    :param robot: the Webots Robot instance owning the devices.
    :return: tuple of the left and right Motor devices.
    """
    leftMotor = robot.getDevice(LEFT_MOTOR_NAME)
    rightMotor = robot.getDevice(RIGHT_MOTOR_NAME)
    for motor in (leftMotor, rightMotor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    return leftMotor, rightMotor


def initialiseProximitySensors(robot, samplingPeriodInMs):
    """Retrieve and enable the eight infrared proximity sensors.

    :param robot: the Webots Robot instance owning the devices.
    :param samplingPeriodInMs: sampling period in milliseconds.
    :return: list of DistanceSensor devices, in ps0 to ps7 order.
    """
    proximitySensors = []
    for sensorName in PROXIMITY_SENSOR_NAMES:
        sensor = robot.getDevice(sensorName)
        sensor.enable(samplingPeriodInMs)
        proximitySensors.append(sensor)
    return proximitySensors


def initialiseLidar(robot, samplingPeriodInMs):
    """Retrieve and enable the two-dimensional LiDAR.

    :param robot: the Webots Robot instance owning the devices.
    :param samplingPeriodInMs: sampling period in milliseconds.
    :return: the enabled Lidar device.
    """
    lidar = robot.getDevice(LIDAR_DEVICE_NAME)
    lidar.enable(samplingPeriodInMs)
    lidar.enablePointCloud()
    return lidar


def formatProximityReport(proximitySensors):
    """Build a one-line summary of the eight proximity readings.

    :param proximitySensors: list of DistanceSensor devices.
    :return: printable string of name and value pairs.
    """
    readings = []
    for sensorName, sensor in zip(PROXIMITY_SENSOR_NAMES, proximitySensors):
        readings.append("%s=%4.0f" % (sensorName, sensor.getValue()))
    return "PROX  " + "  ".join(readings)


def formatLidarReport(lidar):
    """Build a one-line summary of the current LiDAR scan.

    Infinite and not-a-number returns are excluded so that the summary
    reports only ranges the sensor actually resolved.

    :param lidar: the enabled Lidar device.
    :return: printable string describing the finite range returns.
    """
    scan = lidar.getRangeImage()
    if not scan:
        return "LIDAR no scan available yet"
    finiteRanges = []
    for rangeInMetres in scan:
        if rangeInMetres == rangeInMetres:
            if rangeInMetres != float("inf"):
                finiteRanges.append(rangeInMetres)
    if not finiteRanges:
        return "LIDAR %d rays, no finite returns" % len(scan)
    meanRange = sum(finiteRanges) / len(finiteRanges)
    nearestIndex = min(range(len(scan)), key=lambda i: scan[i])
    quarter = len(scan) // 4
    cardinals = []
    for index in (quarter, 2 * quarter, 3 * quarter):
        cardinals.append("r%d=%.3f" % (index, scan[index]))
    blindIndices = [
        index
        for index, value in enumerate(scan)
        if value < SELF_RETURN_THRESHOLD_IN_METRES
    ]
    cardinals.append("selfHits=%d" % len(blindIndices))
    return "LIDAR %d/%d finite  min=%.4f m at ray %d  max=%.3f m  mean=%.3f m  %s" % (
        len(finiteRanges),
        len(scan),
        min(finiteRanges),
        nearestIndex,
        max(finiteRanges),
        meanRange,
        "  ".join(cardinals),
    )


def printConfiguration(samplingPeriodInMs, lidar):
    """Print the device configuration the hand-off note documents.

    :param samplingPeriodInMs: sampling period in milliseconds.
    :param lidar: the enabled Lidar device.
    :return: None.
    """
    print("SMOKE TEST: temporary validation code, not assessed.")
    print("SMOKE TEST: sampling period %d ms." % samplingPeriodInMs)
    print(
        "SMOKE TEST: lidar %d rays x %d layer(s), fov %.4f rad."
        % (lidar.getHorizontalResolution(), lidar.getNumberOfLayers(), lidar.getFov())
    )
    print(
        "SMOKE TEST: lidar range %.3f m to %.3f m."
        % (lidar.getMinRange(), lidar.getMaxRange())
    )


def runSmokeTest():
    """Drive forward briefly while reporting both sensor types.

    :return: None.
    """
    robot = Robot()
    samplingPeriodInMs = int(robot.getBasicTimeStep())
    leftMotor, rightMotor = initialiseMotors(robot)
    proximitySensors = initialiseProximitySensors(robot, samplingPeriodInMs)
    lidar = initialiseLidar(robot, samplingPeriodInMs)
    printConfiguration(samplingPeriodInMs, lidar)

    leftMotor.setVelocity(DRIVE_SPEED_IN_RADIANS_PER_SECOND)
    rightMotor.setVelocity(DRIVE_SPEED_IN_RADIANS_PER_SECOND)

    elapsedInSeconds = 0.0
    nextReportInSeconds = 0.0
    while robot.step(samplingPeriodInMs) != -1:
        if elapsedInSeconds >= nextReportInSeconds:
            print(
                "t=%.2f s  %s"
                % (elapsedInSeconds, formatProximityReport(proximitySensors))
            )
            print("t=%.2f s  %s" % (elapsedInSeconds, formatLidarReport(lidar)))
            nextReportInSeconds += REPORT_INTERVAL_IN_SECONDS
        elapsedInSeconds += samplingPeriodInMs / 1000.0
        if elapsedInSeconds >= DRIVE_DURATION_IN_SECONDS:
            break

    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    robot.step(samplingPeriodInMs)
    print("t=%.2f s  %s" % (elapsedInSeconds, formatProximityReport(proximitySensors)))
    print("t=%.2f s  %s" % (elapsedInSeconds, formatLidarReport(lidar)))
    print("SMOKE TEST: complete, robot stopped, no faults raised.")


if __name__ == "__main__":
    runSmokeTest()
