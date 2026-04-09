import math
from logging import Logger
from typing import Tuple
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import (
    SwerveModuleState,
    SwerveModulePosition,
    ChassisSpeeds,
    SwerveDrive4Kinematics,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
)
from wpimath.units import (
    radiansToRotations,
    rotationsToRadians,
    rotationsToDegrees,
    meters_per_second,
    meters,
)

from .sds import MK4C_L3_GEARING, WHEEL_DIAMETER

from .ctre import (
    MOTOR_OUTPUT_CCWP_BRAKE,
    MOTOR_OUTPUT_CWP_BRAKE,
    MAGNET_CONFIG_CONTWRAP_CCWP,
    FROGPigeonGyro,
    FROGTalonFX,
    FROGCanCoder,
    get_frog_talon_config,
    get_frog_cancoder_config,
)

from .utils import DriveTrain
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
from phoenix6.configs.config_groups import ClosedLoopGeneralConfigs
from wpilib import Timer
from dataclasses import dataclass, field
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue

from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians


@dataclass
class SwerveModuleConfig:

    def __init__(
        self,
        name: str = "undefined",
        location: Translation2d = field(default_factory=Translation2d),
        drive_motor_id: int = 0,
        steer_motor_id: int = 0,
        cancoder_id: int = 0,
        drive_motor_config: TalonFXConfiguration = get_frog_talon_config(),
        steer_motor_config: TalonFXConfiguration = get_frog_talon_config(),
        cancoder_config: CANcoderConfiguration = get_frog_cancoder_config(),
        wheel_diameter: float = 0.0,
    ):
        """Config parameters for the individual swerve modules

        Args:
            name (str, optional): The name of the module, used for network tables. Defaults to "undefined".
            location (Translation2d, optional): Position of the module from the center of the robot. Defaults to field(default_factory=Translation2d).
            drive_motor_id (int, optional): CAN ID of the drive motor. Defaults to 0.
            steer_motor_id (int, optional): CAN ID of the steer motor. Defaults to 0.
            cancoder_id (int, optional): CAN ID of the CANCoder. Defaults to 0.
        """
        self.name = name
        self.location = location
        self.drive_motor_id = drive_motor_id
        self.steer_motor_id = steer_motor_id
        self.cancoder_id = cancoder_id
        self.drive_motor_config = drive_motor_config
        self.steer_motor_config = steer_motor_config
        self.cancoder_config = cancoder_config
        self.wheel_diameter = wheel_diameter


class RotationControllerConfig:
    def __init__(
        self,
        kProfiledRotationP: float = 0.0,
        kProfiledRotationI: float = 0.0,
        kProfiledRotationD: float = 0.0,
        kProfiledRotationMaxVelocity: float = 0.0,
        kProfiledRotationMaxAccel: float = 0.0,
    ):
        self.kProfiledRotationP = kProfiledRotationP
        self.kProfiledRotationI = kProfiledRotationI
        self.kProfiledRotationD = kProfiledRotationD
        self.kProfiledRotationMaxVelocity = kProfiledRotationMaxVelocity
        self.kProfiledRotationMaxAccel = kProfiledRotationMaxAccel


class SwerveModule:
    def __init__(
        self,
        module_config: SwerveModuleConfig,
        parent_nt="Undefined",
    ):
        """Creates a Swerve Module

        Args:
            module_config (SwerveModuleConfig, optional): Configuration for this swerve module.
                Defaults to SwerveModuleConfig().
            parent_nt (str, optional): parent Network Table to place this device under.
                Defaults to "Undefined".
        """
        # set module name
        self.name = module_config.name
        nt_table = f"{parent_nt}/{self.name}"

        # create/configure drive motor
        self.drive_motor = FROGTalonFX(
            id=module_config.drive_motor_id,
            motor_config=module_config.drive_motor_config,
            motor_name=f"{self.name} Drive",
            signal_profile=FROGTalonFX.SignalProfile.SWERVE_DRIVE,
        )

        # create/configure cancoder
        self.steer_encoder = FROGCanCoder(
            id=module_config.cancoder_id, config=module_config.cancoder_config
        )

        # create/configure steer motor using a feedback config that uses the steer encoder as a remote sensor
        self.steer_motor = FROGTalonFX(
            id=module_config.steer_motor_id,
            motor_config=module_config.steer_motor_config.with_feedback(
                FeedbackConfigs()
                .with_feedback_remote_sensor_id(self.steer_encoder.device_id)
                .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
            ),
            motor_name=f"{self.name} Steer",
            signal_profile=FROGTalonFX.SignalProfile.SWERVE_AZIMUTH,
        )

        #  cancoder, update status signal frequency - need absolute position
        self.steer_encoder.get_absolute_position().set_update_frequency(100)
        self.steer_encoder.optimize_bus_utilization()

        # set module location
        self.location = module_config.location
        self.wheel_diameter = module_config.wheel_diameter
        # initialize the state of the module as disabled
        self.enabled = False

        # publish all values as children of the specific swerve module
        # log various values to network tables
        self._moduleCommandedVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/commanded_mps")
            .publish()
        )
        self._moduleCommandedAnglePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/commanded_degrees")
            .publish()
        )
        self._moduleActualVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/actual_mps")
            .publish()
        )
        self._moduleActualAnglePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/actual_degrees")
            .publish()
        )
        self._module_velocity_error_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/velocity_error")
            .publish()
        )
        self._module_angle_error_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/angle_error")
            .publish()
        )

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def getEncoderAzimuthRotations(self):
        """gets the absolute position from the CANCoder
        Returns:
            rotation: absolute position of the sensor in rotations
        """
        return self.steer_encoder.get_absolute_position().value

    def getCurrentSteerAzimuth(self) -> Rotation2d:
        """Gets the Azimuth of the swerve wheel.

        Returns:
            Rotation2d: The robot-relative Azimuth of the swerve wheel.
        """
        if rotations := self.getEncoderAzimuthRotations():
            return Rotation2d(rotationsToRadians(rotations))
        else:
            return Rotation2d(0)

    def getCurrentDistance(self) -> meters:
        """Gets distance traveled by the system.

        Returns:
            meters: distance in meters
        """
        return self.drive_motor.get_position().value

    def getCurrentSpeed(self) -> meters_per_second:
        """Gets the current speed of the drive motor.

        Returns:
            meters_per_second: speed in meters per second
        """
        return self.drive_motor.get_velocity().value

    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentSteerAzimuth(),
        )

    def getCurrentPosition(self):
        return SwerveModulePosition(
            self.getCurrentDistance(), self.getCurrentSteerAzimuth()
        )

    def apply_state(self, requested_state: SwerveModuleState):
        if self.enabled:
            # log the current state of the motors before commanding them to a new value
            # log actual velocity (rps) and position (rotations)
            self.current_velocity = self.getCurrentSpeed()
            self.current_angle = self.getCurrentSteerAzimuth()
            self._moduleActualVelocityPub.set(self.current_velocity)
            self._moduleActualAnglePub.set(self.current_angle.degrees())

            # update the state with the steer angle that is closest to the current angle
            requested_state.optimize(self.current_angle)

            # command the steer motor to the requested angle
            self.commandedRotation = radiansToRotations(requested_state.angle.radians())
            self.steer_motor.set_control(
                PositionVoltage(
                    position=self.commandedRotation,
                    slot=0,  # Position voltage gains for steer
                )
            )
            self._moduleCommandedAnglePub.set(
                rotationsToDegrees(self.commandedRotation)
            )

            self.commandedSpeed = requested_state.speed
            self.drive_motor.set_control(
                VelocityVoltage(
                    velocity=self.commandedSpeed,
                    slot=0,  # Voltage gains for drive
                )
            )
            self._moduleCommandedVelocityPub.set(self.commandedSpeed)

            # publish the error for velocicty and anble
            self._module_velocity_error_pub.set(
                self.drive_motor.get_closed_loop_error().value
            )
            self._module_angle_error_pub.set(
                self.drive_motor.get_closed_loop_error().value
            )

        else:
            # stop the drive motor, steer motor can stay where it is
            self.drive_motor.set_control(VelocityVoltage(velocity=0, slot=1))
        # self.steer.logData()


class SwerveChassis:

    def __init__(
        self,
        swerve_module_configs: tuple[SwerveModuleConfig],
        gyro: FROGPigeonGyro,
        rotation_contoller_config: RotationControllerConfig,
        max_speed: float,
        max_rotation_speed: float,
        parent_nt: str = "Undefined",
    ):
        # set the name of this component to the class name
        self.name = self.__class__.__name__
        # set the network tables path for this component
        nt_table = f"{parent_nt}/{self.name}"

        # Swerve components
        #####################################

        # instantiate all 4 swerve modules.  Using a tuple to preserve the order of the modules.
        # Convention is to list them in the following order:
        # Front Left, Front Right, Back Left, Back Right
        self.modules = tuple(
            SwerveModule(config, parent_nt=nt_table) for config in swerve_module_configs
        )
        self.gyro = gyro

        self.swerve_kinematics = SwerveDrive4Kinematics(
            # the splat operator (asterisk) below expands
            # the list into positional arguments for the
            # kinematics object.  We are taking the location
            # property of each swerveModule object and passing
            # it to SwerveDrive4Kinematics the order defined by
            # self.modules above.  Order is critical here.
            # We will receive back drive and steer values for
            # each SwerveModule in the same order we use here.
            *[m.location for m in self.modules]
        )
        # the swerve estimator needs to know the initial position of each module to start tracking the pose of the robot.
        # We start with an initial distance of 0 for each module and the current steer azimuth.
        initial_module_positions = tuple(
            [SwerveModulePosition(0, x.getCurrentSteerAzimuth()) for x in self.modules]
        )
        self.swerve_estimator = SwerveDrive4PoseEstimator(
            self.swerve_kinematics,
            self.gyro.getRotation2d(),
            initial_module_positions,
            Pose2d(),  # Initial pose set to empty by design; vision initialization handled separately if needed.
        )

        # Swerve Characteristics
        #####################################

        # define the center of rotation for the robot.  All swerve modules positions must be given in relation to this point.
        self.center = Translation2d(0, 0)
        self.max_speed = max_speed
        self.max_rotation_speed = max_rotation_speed

        # Swerve initial variables
        #####################################

        # creates a tuple of 4 SwerveModuleState objects
        # this attribute will hold the desired state (speed and direction) for each module
        self.moduleStates = (SwerveModuleState(),) * 4
        # this attribute will hold the requested speeds of the robot chassis (x, y, and rotational)
        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)
        # start with the chassis disabled
        self.enabled = False

        # initialize timer for loop time calculations
        self.timer = Timer()
        self.timer.start()
        self.lastTime = self.timer.get()
        self.loopTime = 0

        # create rotation controller for auto-rotation
        self.profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            rotation_contoller_config.kProfiledRotationMaxVelocity,
            rotation_contoller_config.kProfiledRotationMaxAccel,
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            rotation_contoller_config.kProfiledRotationP,
            rotation_contoller_config.kProfiledRotationI,
            rotation_contoller_config.kProfiledRotationD,
            self.profiledRotationConstraints,
        )
        self.profiledRotationController.enableContinuousInput(-math.pi, math.pi)

        # Network Tables publishers
        #####################################
        self._chassisSpeedsPub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/chassisSpeedsCommanded", ChassisSpeeds)
            .publish()
        )
        self._chassisSpeedsActualPub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/chassisSpeedsActual", ChassisSpeeds)
            .publish()
        )
        self._chassisSpeedsErrorPub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/chassisSpeedsError", ChassisSpeeds)
            .publish()
        )
        self._estimatorPosePub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/estimatorPose", Pose2d)
            .publish()
        )
        # create NT publisher for boolean odding vision measurements to the swerve estimator
        self._useVisionMeasurementsPub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/useVisionMeasurements")
            .publish()
        )

    def disable(self):
        self.enabled = False
        for module in self.modules:
            module.disable()

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    # TODO: #4 Consolidate fieldOrientedDrive and fieldOrientedAutoRotateDrive
    # add a boolean parameter to indicate whether to apply throttle to rotation or not
    def fieldOrientedDrive(self, vX: float, vY: float, vT: float, throttle=1.0):
        """Calculates the necessary chassis speeds given the commanded field-oriented
        x, y, and rotational speeds.  An optional throttle value adjusts all inputs
        proportionally.

        Args:
            vX (float): velocity requested in the X direction, downfield, away from
                the driver station.  A proportion of the maximum speed.  (-1 to 1)
            vY (float): velocity requested in the Y direction, to the left when at the
                driver station facing the field.  A proportion of the maximum speed.  (-1 to 1)
            vT (float): rotational velocity requested, CCW positive (-1 to 1)
            throttle (float, optional): a proportion of all 3 speeds commanded.
                Defaults to 1.0.
        """
        xSpeed = vX * self.max_speed * throttle
        ySpeed = vY * self.max_speed * throttle
        rotSpeed = vT * self.max_rotation_speed * throttle
        self.apply_chassis_speeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, self.getRotation2d()
            )
        )

    def fieldOrientedAutoRotateDrive(
        self, vX: float, vY: float, vT: float, throttle=1.0
    ):
        """Calculates the necessary chassis speeds given the commanded field-oriented
        x, y, and rotational speeds.  An optional throttle value adjusts only the x
        and y speeds.  The rotational speed is not affected by the throttle.

        Args:
            vX (float): velocity requested in the X direction, downfield, away from
                the driver station.  A proportion of the maximum speed.  (-1 to 1)
            vY (float): velocity requested in the Y direction, to the left when at the
                driver station facing the field.  A proportion of the maximum speed.  (-1 to 1)
            vT (float): rotational velocity requested, CCW positive (-1 to 1)
            throttle (float, optional): a proportion of the x and y speeds commanded.
                Defaults to 1.0.
        """
        xSpeed = vX * self.max_speed * throttle
        ySpeed = vY * self.max_speed * throttle
        rotSpeed = vT * self.max_rotation_speed
        self.apply_chassis_speeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, self.getRotation2d()
            )
        )

    def fieldOrientedDriveWithHeading(
        self, vX: float, vY: float, target_heading: float, throttle: float = 1.0
    ) -> None:
        """
        Calculates the necessary chassis speeds for field-oriented driving with a target heading (rotation).
        Uses the profiled rotation controller to generate rotational velocity to reach the target angle.
        Throttle applies only to translational speeds (vX, vY); rotation is profiled independently.

        Args:
            vX (float): Velocity requested in the X direction (downfield), proportion of max speed (-1 to 1).
            vY (float): Velocity requested in the Y direction (left), proportion of max speed (-1 to 1).
            target_heading (float): Desired robot heading in radians (field-relative, continuous -π to π).
            throttle (float, optional): Proportion applied to vX and Vy (translational). Defaults to 1.0.
        """
        # Compute translational speeds (throttled)
        x_speed = vX * self.max_speed * throttle
        y_speed = vY * self.max_speed * throttle

        # Get current heading in radians
        current_heading = self.getRotation2d().radians()

        # Use profiled controller to compute rotational velocity towards target
        rot_speed = self.profiledRotationController.calculate(
            current_heading, target_heading
        )

        # Create field-relative chassis speeds
        robot_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed, y_speed, rot_speed, self.getRotation2d()
        )

        # Apply the speeds
        self.apply_chassis_speeds(robot_speeds)

    def calculateHeadingToTarget(self, target_position: Pose2d) -> float:
        """
        Calculates the heading angle (in radians) from the robot's current position to a target position.
        Args:
            target_position (Pose2d): The target position on the field.
        Returns:
            float: The angle in radians from the robot's current position to the target position.
        """
        current_pose = self.getPose()
        calculated_heading = (
            target_position.translation() - current_pose.translation()
        ).angle()
        return calculated_heading.radians()

    def getActualChassisSpeeds(self):
        return self.swerve_kinematics.toChassisSpeeds(self.getModuleStates())

    def getChassisVelocityFPS(self):
        return math.sqrt(self.chassisSpeeds.vx_fps**2 + self.chassisSpeeds.vy_fps**2)

    def getHeadingRadians(self):
        return math.atan2(self.chassisSpeeds.vy, self.chassisSpeeds.vx)

    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]

    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    # Returns the current pose of the robot as a Pose2d.
    def getPose(self) -> Pose2d:
        # translation = self.estimator.getEstimatedPosition().translation()
        # rotation = self.gyro.getRotation2d()
        # return Pose2d(translation, rotation)
        return self.swerve_estimator.getEstimatedPosition()

    # Returns a ChassisSpeeds object representing the speeds in the robot's frame
    # of reference.
    def getRobotRelativeSpeeds(self):
        return self.chassisSpeeds

    def getRotation2d(self) -> Rotation2d:
        return self.getPose().rotation()

    def lockChassis(self):
        # getting the "angle" of each module location on the robot.
        # this gives us the angle back to the center of the robot from
        # the module
        moduleAngles = [y.location.angle() for y in self.modules]
        # now we tell each module to steer the wheel to that angle.
        # with each angle turned to the center of the robot, the chassis
        # is effectively "locked" in position on the field.
        for module, moduleAngle in zip(self.modules, moduleAngles):
            module.apply_state(SwerveModuleState(0, moduleAngle))

    def logTelemetry(self):
        self._actualChassisSpeeds = self.getActualChassisSpeeds()
        self._chassisSpeedsActualPub.set(self._actualChassisSpeeds)
        self._chassisSpeedsPub.set(self.chassisSpeeds)
        self._chassisSpeedsErrorPub.set(self.chassisSpeeds - self._actualChassisSpeeds)
        self._estimatorPosePub.set(self.swerve_estimator.getEstimatedPosition())

    def periodic(self):
        self.update_loop_time()

        # if self.enabled:
        #     self._setStatesFromSpeeds()  # apply chassis Speeds
        #     for module, state in zip(self.modules, self.moduleStates):
        #         module.apply_state(state)

        self.logTelemetry()

    def update_loop_time(self):
        self.newTime = self.timer.get()
        self.loopTime = self.newTime - self.lastTime
        self.lastTime = self.newTime

    def get_loop_time(self):
        return self.loopTime

    # Resets the pose by running the resetPosition method of the estimator.
    def resetPose(self, pose: Pose2d):
        self.swerve_estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def robotOrientedDrive(self, vX, vY, vT, throttle=1.0):
        xSpeed = vX * self.max_speed * throttle
        ySpeed = vY * self.max_speed * throttle
        rotSpeed = vT * self.max_rotation_speed
        self.apply_chassis_speeds(ChassisSpeeds(xSpeed, ySpeed, rotSpeed))

    # def setChassisSpeeds(self, speeds: ChassisSpeeds):
    #     """Give the swerve drive the latest discretized ChassisSpeeds for it to apply.

    #     Args:
    #         speeds (ChassisSpeeds): The ChassisSpeeds to apply.
    #     """
    #     self.chassisSpeeds = ChassisSpeeds.discretize(speeds, self.get_loop_time())

    # def _setModuleStates(self, states):
    #     self.moduleStates = states

    # def _setStatesFromSpeeds(self):
    #     """Calculates and sets the module states from the current chassis speeds."""

    #     states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
    #     states = self.kinematics.desaturateWheelSpeeds(states, self.max_speed)
    #     self._setModuleStates(states)

    def apply_chassis_speeds(self, speeds: ChassisSpeeds):
        """Applies the given chassis speeds to the swerve drive.

        Args:
            speeds (ChassisSpeeds): The chassis speeds to apply.
        """
        self.chassisSpeeds = ChassisSpeeds.discretize(speeds, self.get_loop_time())
        states = self.swerve_kinematics.toSwerveModuleStates(
            self.chassisSpeeds, self.center
        )

        if self.enabled:
            # apply module states
            for module, state in zip(self.modules, states):
                module.apply_state(state)
