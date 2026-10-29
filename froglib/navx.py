"""NavX gyro wrapper providing field-oriented and swerve-compatible angular telemetry.

This module wraps the Kauai Labs NavX AHRS sensor to provide angular orientation,
rates of rotation, and alliance-specific reset capabilities with counter-clockwise (CCW)
positive coordinate conventions required for swerve drivetrains and odometry.
"""

import math
from navx import AHRS
from wpimath.geometry import Rotation2d


class FROGNavXGyro:
    """Gyro class that creates an instance of the NavX gyro and uses it to get AHRS data,
    converting it for use by the swerve drivetrain. All swerve calculations use radians
    with CCW rotation being positive, and field-oriented driving uses moving forward and
    moving left as positive.

    Attributes:
        gyro (AHRS): The underlying Kauai Labs NavX AHRS instance created over SPI.
        starting_angle (float): Initial reference angle in degrees.
        offset (float): Angle adjustment offset in degrees applied to the gyro.
    """

    def __init__(self):
        """Initializes the NavX gyro over SPI and applies the initial angle adjustment."""
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        self.gyro.setAngleAdjustment(self.offset)

    def getAngleCCW(self) -> float:
        """Gets the current gyro heading with counter-clockwise rotation positive.

        Inverts the NavX bearing angle to convert from compass heading to Cartesian
        angles where CCW is positive.

        Returns:
            float: Gyro heading in degrees with CCW positive.
        """
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return -self.gyro.getAngle()

    def getRoll(self) -> float:
        """Gets the current roll angle in degrees from the AHRS.

        Returns:
            float: Roll angle in degrees.
        """
        return self.gyro.getRoll()

    def getPitch(self) -> float:
        """Gets the current pitch angle in degrees from the AHRS.

        Returns:
            float: Pitch angle in degrees.
        """
        return self.gyro.getPitch()

    def setOffset(self, offset: float):
        """Sets the internal heading offset angle.

        Args:
            offset (float): Offset angle in degrees.
        """
        self.offset = offset

    def getDegreesPerSecCCW(self) -> float:
        """Gets the rate of rotation in degrees per second with CCW positive.

        Returns:
            float: Rate of angular rotation in degrees per second.
        """
        return -self.gyro.getRate()

    def getRadiansPerSecCCW(self) -> float:
        """Gets the rate of rotation in radians per second with CCW positive.

        Returns:
            float: Rate of angular rotation in radians per second.
        """
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self) -> Rotation2d:
        """Gets the current robot heading as a WPILib Rotation2d object.

        Returns:
            Rotation2d: Current heading wrapped as Rotation2d.
        """
        return Rotation2d.fromDegrees(self.getAngleCCW())

    def resetGyro(self, on_red: bool):
        """Resets the gyro yaw reading to zero, taking alliance orientation into account.

        Args:
            on_red (bool): True if currently on the Red alliance (sets 180° adjustment),
                False if on the Blue alliance (sets 0° adjustment).
        """
        # sets yaw reading to 0
        if on_red:
            self.setAngleAdjustment(180)
        else:
            self.setAngleAdjustment(0)
        self.gyro.reset()

    def getAngleConstrained(self) -> float:
        """Gets the current angle constrained to the range [-180, 180] degrees.

        Returns:
            float: Constrained angle in degrees.
        """
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle: float):
        """Sets the NavX internal angle adjustment.

        Args:
            angle (float): Adjustment angle in degrees.
        """
        self.gyro.setAngleAdjustment(angle)

    def getAngleAdjustment(self) -> float:
        """Gets the current NavX internal angle adjustment.

        Returns:
            float: Adjustment angle in degrees.
        """
        return self.gyro.getAngleAdjustment()
