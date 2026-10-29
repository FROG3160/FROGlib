"""Motor controller wrappers and drive train helpers for FROGlib.

Note:
    This module provides legacy motor abstractions (Phoenix 5 TalonSRX and early
    Phoenix 6 TalonFX wrappers) along with gearing calculation helpers. For modern
    Phoenix 6 development in 2026/2027, use `froglib.ctre` which provides
    advanced signal update profiling, automated characterization, and hardware simulation.
    For modern gearing representations, prefer `froglib.utils`.
"""

import math
from ntcore import NetworkTableInstance
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import GravityTypeValue

# from rev import CANSparkMax, SparkAbsoluteEncoder
from phoenix5 import StatusFrameEnhanced, TalonSRX, TalonSRXConfiguration


class FROGFeedbackConfig(FeedbackConfigs):
    """FROG subclass of FeedbackConfigs that allows parameter passing during instantiation."""

    def __init__(
        self, remote_sensor_id=0, sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR
    ):
        """Initializes a FROGFeedbackConfig with sensor parameters.

        Args:
            remote_sensor_id (int, optional): CAN ID of the remote sensor. Defaults to 0.
            sensor_source (FeedbackSensorSourceValue, optional): Source value from
                FeedbackSensorSourceValue. Defaults to FeedbackSensorSourceValue.ROTOR_SENSOR.
        """
        super().__init__()
        self.feedback_remote_sensor_id = remote_sensor_id
        self.feedback_sensor_source = sensor_source
        # TODO: Add this in if it makes sense
        # self.rotor_to_sensor_ratio =
        # self.sensor_to_mechanism_ratio =


class FROGTalonFXConfig(TalonFXConfiguration):
    """FROG subclass of TalonFXConfiguration with inline gain and feedback initialization."""

    def __init__(
        self,
        feedback_config=FROGFeedbackConfig(),
        slot0gains=Slot0Configs(),
        slot1gains=Slot1Configs(),
    ):
        """Initializes a FROGTalonFXConfig with feedback and gain slot configurations.

        Args:
            feedback_config (FROGFeedbackConfig, optional): Feedback configuration for the motor.
                Defaults to FROGFeedbackConfig().
            slot0gains (Slot0Configs, optional): Slot 0 PID and feedforward gains.
                Defaults to Slot0Configs().
            slot1gains (Slot1Configs, optional): Slot 1 PID and feedforward gains.
                Defaults to Slot1Configs().
        """
        super().__init__()
        self.feedback = feedback_config
        self.slot0 = slot0gains
        self.slot1 = slot1gains


class FROGTalonSRXConfig(TalonSRXConfiguration):
    """FROG subclass of Phoenix 5 TalonSRXConfiguration."""

    def __init__(self):
        """Initializes a default TalonSRXConfiguration."""
        super().__init__()


class FROGTalonFX(TalonFX):
    """A FROG subclass of TalonFX that allows passing in configuration during instantiation.

    Note:
        For new robot code, consider using `froglib.ctre.FROGTalonFX` which supports
        update frequency profiling and automated characterization.
    """

    def __init__(
        self,
        id: int = 0,
        motor_config: FROGTalonFXConfig = FROGTalonFXConfig(),
        parent_nt: str = "Undefined",
        motor_name: str = "",
    ):
        """Creates a TalonFX motor object with applied configuration and telemetry topics.

        Args:
            id (int, optional): The CAN ID assigned to the motor. Defaults to 0.
            motor_config (FROGTalonFXConfig, optional): The configuration to apply to the motor.
                Defaults to FROGTalonFXConfig().
            parent_nt (str, optional): NetworkTables parent path for motor telemetry.
                Defaults to "Undefined".
            motor_name (str, optional): NetworkTables motor name.
                Defaults to "" (generates "TalonFX({id})").
        """
        super().__init__(device_id=id)
        self.config = motor_config
        self.configurator.apply(self.config)
        if motor_name == "":
            motor_name = f"TalonFX({id})"
        table = f"{parent_nt}/{motor_name}"
        self._motorVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/velocity")
            .publish()
        )
        self._motorPositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/position")
            .publish()
        )
        self._motorVoltagePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/voltage")
            .publish()
        )

    def getMotorVoltage(self) -> float:
        """Retrieves the applied motor output voltage.

        Returns:
            float: Applied output voltage in Volts.
        """
        return self.get_motor_voltage().value

    def logData(self) -> None:
        """Publishes motor velocity telemetry to NetworkTables."""
        self._motorVelocityPub.set(self.get_velocity().value)
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)


class FROGTalonSRX(TalonSRX):
    """A FROG subclass of TalonSRX that allows passing in configuration during instantiation."""

    def __init__(
        self,
        id: int = 0,
        motor_config: FROGTalonSRXConfig = FROGTalonSRXConfig(),
        parent_nt: str = "Undefined",
        motor_name: str = "",
    ):
        """Creates a TalonSRX motor object with applied configuration and telemetry topics.

        Args:
            id (int, optional): The CAN ID assigned to the motor. Defaults to 0.
            motor_config (FROGTalonSRXConfig, optional): The configuration to apply to the motor.
                Defaults to FROGTalonSRXConfig().
            parent_nt (str, optional): NetworkTables parent path for motor telemetry.
                Defaults to "Undefined".
            motor_name (str, optional): NetworkTables motor name.
                Defaults to "" (generates "TalonSRX({id})").
        """
        super().__init__(deviceNumber=id)
        self.config = motor_config
        self.configAllSettings(self.config)
        self.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250)

        if motor_name == "":
            motor_name = f"TalonSRX({id})"
        table = f"{parent_nt}/{motor_name}"
        self._motorVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/velocity")
            .publish()
        )
        self._motorPositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/position")
            .publish()
        )
        self._motorVoltagePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/voltage")
            .publish()
        )

    def getMotorVoltage(self) -> float:
        """Retrieves the applied motor output voltage.

        Returns:
            float: Output voltage in Volts.
        """
        return self.getMotorOutputVoltage()

    def logData(self) -> None:
        """Publishes motor telemetry to NetworkTables."""
        pass
        # self._motorVelocityPub.set(self.get_velocity().value)
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)


class GearTrain:
    """Helper class for computing gear reduction ratios across multiple stages."""

    def __init__(self, gear_stages: list):
        """Constructs a GearTrain object that stores data about the gear stages.

        Args:
            gear_stages (list): List of gear stages expressed as ratios or tuples of two integers
                (driving_teeth, driven_teeth), e.g. [(10, 32), (9, 24)].
        """
        self.gear_ratio = math.prod(gear_stages)

    def input_rotations(self, output_rotations: float) -> float:
        """Calculates input (motor) rotations from mechanism output rotations.

        Args:
            output_rotations (float): Output mechanism rotations.

        Returns:
            float: Required input motor rotations.
        """
        return output_rotations / self.gear_ratio

    def output_rotations(self, input_rotations: float) -> float:
        """Calculates final mechanism rotations from input (motor) rotations.

        Args:
            input_rotations (float): Input motor rotations.

        Returns:
            float: Resulting output mechanism rotations.
        """
        return input_rotations * self.gear_ratio


class DriveTrain:
    """Helper class combining gear reduction and wheel geometry for drivetrain calculations."""

    def __init__(self, gear_stages: list, wheel_diameter: float):
        """Constructs a DriveTrain object that stores data about the gear stages and wheel.

        Args:
            gear_stages (list): List of gear stages expressed as ratios or stage tuples.
            wheel_diameter (float): Diameter of the attached wheel in meters.
        """
        self.gearing = GearTrain(gear_stages)
        self.circumference = math.pi * wheel_diameter

    def speed_to_input_rps(self, speed: float) -> float:
        """Converts linear robot speed to motor rotational velocity.

        Args:
            speed (float): Desired linear speed in meters per second.

        Returns:
            float: Motor velocity in rotations per second (RPS).
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.input_rotations(wheel_rotations_sec)
        return motor_rotations_sec

    def input_rps_to_speed(self, rotations_per_sec: float) -> float:
        """Converts motor rotational velocity to linear robot speed.

        Args:
            rotations_per_sec (float): Motor rotational velocity in rotations per second (RPS).

        Returns:
            float: Linear speed in meters per second.
        """
        wheel_rotations_sec = self.gearing.output_rotations(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def rotations_to_distance(self, rotations: float) -> float:
        """Converts motor rotations into linear distance traveled.

        Args:
            rotations (float): Number of motor rotations.

        Returns:
            float: Linear distance traveled in meters.
        """
        wheel_rotations = self.gearing.output_rotations(rotations)
        return wheel_rotations * self.circumference


# class FROGSparkMax(CANSparkMax):
#     def __init__(
#         self, id, motor_type, parent_nt: str = "Undefined", motor_name: str = ""
#     ):
#         super().__init__(id, motor_type)
#         self.encoder = self.getEncoder()
# 
#         if motor_name == "":
#             motor_name = f"SparkMax({id})"
#         table = f"{parent_nt}/{motor_name}"
# 
#         self._motorVelocityPub = (
#             NetworkTableInstance.getDefault()
#             .getFloatTopic(f"{table}/velocity")
#             .publish()
#         )
#         self._motorPositionPub = (
#             NetworkTableInstance.getDefault()
#             .getFloatTopic(f"{table}/position")
#             .publish()
#         )
#         self._motorVoltagePub = (
#             NetworkTableInstance.getDefault()
#             .getFloatTopic(f"{table}/voltage")
#             .publish()
#         )
# 
#     def getMotorVoltage(self):
#         return self.getAppliedOutput()
# 
#     def logData(self):
#         """Logs data to network tables for this motor"""
#         pass
#         # self._motorVelocityPub.set(self.encoder.getVelocity())
#         # self._motorPositionPub.set(self.encoder.getPosition())
#         # self._motorVoltagePub.set(
#         #     self.getAppliedOutput()
#         # )  # not sure if this is right or not
