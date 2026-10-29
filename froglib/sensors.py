"""Sensor wrappers and configurations for FRC robot subsystems.

This module provides hardware abstraction wrappers for sensors commonly used in FRC,
specifically the Kauai Labs NavX AHRS sensor (`FROGGyro`) and CTRE Phoenix 6 CANcoder
absolute magnetic encoders (`FROGCanCoder`, `FROGCANCoderConfig`). These wrappers standardize
coordinate systems, directional conventions (counter-clockwise positive), and alliance-aware
resets for swerve drivetrains and mechanism angle sensing.
"""

import math
from navx import AHRS
from wpimath.geometry import Rotation2d
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.signals.spn_enums import AbsoluteSensorRangeValue, SensorDirectionValue
from phoenix6.hardware.cancoder import CANcoder


class FROGGyro:
    """Gyro class that creates an instance of the NavX gyro and uses it to get AHRS data,
    converting it for use by the swerve drivetrain. All swerve calculations use radians
    with CCW rotation being positive, and field-oriented driving uses moving forward and
    moving left as positive.

    Attributes:
        gyro (AHRS): The underlying Kauai Labs NavX AHRS instance.
        starting_angle (float): Initial reference angle in degrees.
        offset (float): Offset angle applied to gyro readings in degrees.
    """

    def __init__(self):
        """Initializes the NavX gyro over SPI and applies the initial angle adjustment."""
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
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


class FROGCANCoderConfig(CANcoderConfiguration):
    """Inherits from CANcoderConfiguration and adds the ability to pass in steer offset
    during instantiation."""

    def __init__(self, steer_offset: float):
        """Initializes a CANcoder configuration with signed range and steer offset.

        Args:
            steer_offset (float): Magnet offset in rotations to calibrate wheel zero heading.
        """
        super().__init__()
        self.magnet_sensor.absolute_sensor_range = (
            AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        )
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )


class FROGCanCoder(CANcoder):
    """CTRE CANcoder absolute magnetic encoder wrapper.

    Specializes Phoenix 6 CANcoder to automatically apply a FROGCANCoderConfig
    upon instantiation.
    """

    def __init__(self, id: int, config: FROGCANCoderConfig):
        """Initializes the CANcoder and applies the configuration.

        Args:
            id (int): CAN bus ID of the CANcoder device.
            config (FROGCANCoderConfig): Configuration containing offset, sensor range, and direction.
        """
        super().__init__(id)
        self.configurator.apply(config)
