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
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue
from wpimath.units import radiansToRotations, rotationsToRadians
from .motors import FROGTalonFX, FROGTalonFXConfig, DriveTrain
from .sensors import FROGCANCoderConfig, FROGCanCoder, FROGGyro
from phoenix6.configs.config_groups import ClosedLoopGeneralConfigs
from wpilib import Timer


class SwerveModule:
    def __init__(
        self,
        name: str,
        location: Translation2d,
        drive_gearing: list,
        wheel_diameter: float,
        drive_motor_id: int,
        drive_motor_config: FROGTalonFXConfig,
        steer_motor_id: int,
        steer_motor_config: FROGTalonFXConfig,
        cancoder_id: int,
        cancoder_config: FROGCANCoderConfig,
        parent_nt="Undefined",
    ):
        """Creates a Swerve Module

        Args:
            name (str): Name of the module
            location (Translation2d): coordinates from the center of the robot.
            drive_gearing (list): the gear stages of the drive wheel
            wheel_diameter (float): the diameter of the drive wheel in meters.
            drive_motor_id (int): CAN ID of the drive motor.
            drive_motor_config (FROGTalonFXConfig): config for the drive motor.
            steer_motor_id (int): CAN ID of the steer motor.
            steer_motor_config (FROGTalonFXConfig): config for the steer motor.
            cancoder_id (int): CAN ID of the CANcoder (steer encoder).
            cancoder_config (FROGCANCoderConfig): config for the CANcoder (steer encoder).
            parent_nt (str, optional): parent Network Table to place this device under.
                Defaults to "Undefined".
        """        # set module name
        self.name = name
        # set neutral mode for drive motor
        drive_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # with the non-inverted SwerveDriveSpecialties swerve modules, and
        # the bevel gears facing left, the drive motors need to be inverted
        # in order to move the drivetrain forward with a positive value.
        # the default inverted setting is CCW positive.
        drive_motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        # create/configure drive motor
        self.drive_motor = FROGTalonFX(
            drive_motor_id,
            drive_motor_config,
            parent_nt=f"{parent_nt}/{self.name}",
            motor_name="Drive",
        )

        # set continuous wrap to wrap around the 180 degree point
        steer_motor_config.closed_loop_general.continuous_wrap = True
        # create/configure steer motor
        self.steer_motor = FROGTalonFX(
            steer_motor_id,
            steer_motor_config,
            parent_nt=f"{parent_nt}/{self.name}",
            motor_name="Steer",
        )

        # create/configure cancoder
        self.steer_encoder = FROGCanCoder(cancoder_id, cancoder_config)

        self.drive_motor.get_velocity().set_update_frequency(50)
        self.drive_motor.get_motor_voltage().set_update_frequency(50)
        self.steer_motor.get_position().set_update_frequency(50)
        self.steer_encoder.get_absolute_position().set_update_frequency(50)
        self.drive_motor.optimize_bus_utilization()
        self.steer_motor.optimize_bus_utilization()
        self.steer_encoder.optimize_bus_utilization()

        # set module location
        self.location = location
        #
        self.drivetrain = DriveTrain(drive_gearing, wheel_diameter)

        self.enabled = False

        nt_table = f"{parent_nt}/{self.name}"
        self._moduleSpeedPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/commanded_speed")
            .publish()
        )
        self._moduleRotationPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/commanded_angle")
            .publish()
        )
        self._moduleVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/actual_velocity")
            .publish()
        )
        self._modulePositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/actual_position")
            .publish()
        )

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def getEncoderAzimuthRotations(self) -> float:
        """gets the absolute position from the CANCoder
        Returns:
            float: absolute position of the sensor in rotations
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

    def getCurrentDistance(self) -> float:
        """Gets distance traveled by the system.

        Returns:
            float: distance in meters
        """
        return self.drivetrain.rotations_to_distance(self.drive_motor.get_position().value)

    def getCurrentSpeed(self) -> float:
        return self.drivetrain.input_rps_to_speed(self.drive_motor.get_velocity().value)

    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentSteerAzimuth(),
        )

    def getCurrentPosition(self):
        return SwerveModulePosition(
            self.getCurrentDistance(), self.getCurrentSteerAzimuth()
        )

    def setState(self, requested_state: SwerveModuleState):
        if self.enabled:
            # log the current state of the motors before commanding them to a new value
            self._moduleVelocityPub.set(self.drive_motor.get_velocity().value)
            self._modulePositionPub.set(self.steer_motor.get_position().value)

            self.requestedState = SwerveModuleState.optimize(
                requested_state, self.getCurrentSteerAzimuth()
            )
            self.commandedRotation = radiansToRotations(
                self.requestedState.angle.radians()
            )
            self.commandedSpeed = self.drivetrain.speed_to_input_rps(
                self.requestedState.speed
            )
            self.steer_motor.set_control(
                PositionDutyCycle(
                    position=self.commandedRotation,
                    slot=0,  # Duty Cycle gains for steer
                )
            )
            self._moduleRotationPub.set(self.commandedRotation)
            self.drive_motor.set_control(
                VelocityVoltage(
                    velocity=self.commandedSpeed,
                    slot=1,  # Voltage gains for drive
                )
            )
            self._moduleSpeedPub.set(self.commandedSpeed)

        else:
            # stop the drive motor, steer motor can stay where it is
            self.drive_motor.set_control(VelocityVoltage(velocity=0, slot=1))
        self.drive_motor.logData()
        # self.steer.logData()


class SwerveBase(Subsystem):
    def __init__(
        self,
        swerve_module_configs,
        gyro: FROGGyro,
        max_speed: float,
        max_rotation_speed: float,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("SwerveChassis")
        nt_table = f"{parent_nt}/{self.getName()}"
        # need each of the swerve modules
        self.enabled = False

        self.center = Translation2d(0, 0)
        self.modules = tuple(
            SwerveModule(**config, parent_nt=nt_table)
            for config in swerve_module_configs
        )
        self.gyro = gyro
        self.max_speed = max_speed
        self.max_rotation_speed = max_rotation_speed

        # creates a tuple of 4 SwerveModuleState objects
        self.moduleStates = (SwerveModuleState(),) * 4

        self.kinematics = SwerveDrive4Kinematics(
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

        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple(
                [
                    SwerveModulePosition(0, x.getCurrentSteerAzimuth())
                    for x in self.modules
                ]
            ),
            Pose2d(),  # TODO:  Determine if we want vision data to supply initial pose
            # last year, setFieldPosition was called and passed the vision pose during
            # robotInit()
        )
        self.timer = Timer()
        self.timer.start()
        self.lastTime = 0
        self.loopTime = 0

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
        self._estimatedPositionPub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/estimatedPosition", Pose2d)
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
        self.chassisSpeeds = ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, self.getRotation2d()
            ),
            self.loopTime,
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
        self.chassisSpeeds = ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, self.getRotation2d()
            ),
            self.loopTime,
        )

    def getActualChassisSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getChassisVelocityFPS(self):
        return math.sqrt(self.chassisSpeeds.vx_fps**2 + self.chassisSpeeds.vy_fps**2)

    def getHeadingRadians(self):
        return math.atan2(self.chassisSpeeds.vy, self.chassisSpeeds.vx)

    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]

    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    # Returns a ChassisSpeeds object representing the speeds in the robot's frame
    # of reference.
    def getRobotRelativeSpeeds(self):
        return self.chassisSpeeds

    # Returns the current pose of the robot as a Pose2d.
    def getPose(self) -> Pose2d:
        # translation = self.estimator.getEstimatedPosition().translation()
        # rotation = self.gyro.getRotation2d()
        # return Pose2d(translation, rotation)
        return self.estimator.getEstimatedPosition()

    def getRotation2d(self) -> Rotation2d:
        return self.getPose().rotation()

    # returns a Pose with rotation flipped
    # SHOULD NO LONGER BE USED when using blue alliance coordintate system all the time
    # def getFlippedPose(self) -> Pose2d:
    #     if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
    #         translation = self.estimator.getEstimatedPosition().translation()
    #         rotation = self.gyro.getRotation2d().rotateBy(Rotation2d(math.pi))
    #         return Pose2d(translation, rotation)
    #     else:
    #         return self.getPose()

    def lockChassis(self):
        # getting the "angle" of each module location on the robot.
        # this gives us the angle back to the center of the robot from
        # the module
        moduleAngles = [y.location.angle() for y in self.modules]
        # now we tell each module to steer the wheel to that angle.
        # with each angle turned to the center of the robot, the chassis
        # is effectively "locked" in position on the field.
        for module, moduleAngle in zip(self.modules, moduleAngles):
            module.setState(SwerveModuleState(0, moduleAngle))

    def logTelemetry(self):
        # self._actualChassisSpeeds = self.getActualChassisSpeeds()
        # self._chassisSpeedsActualPub.set(self._actualChassisSpeeds)
        # self._chassisSpeedsPub.set(self.chassisSpeeds)
        # self._chassisSpeedsErrorPub.set(self.chassisSpeeds - self._actualChassisSpeeds)
        self._estimatedPositionPub.set(self.estimator.getEstimatedPosition())

    def periodic(self):
        self.newTime = self.timer.get()
        self.loopTime = self.newTime - self.lastTime
        self.lastTime = self.newTime

        if self.enabled:
            self.setStatesFromSpeeds()  # apply chassis Speeds
        for module, state in zip(self.modules, self.moduleStates):
            module.setState(state)

        self.logTelemetry()

    # Resets the pose by running the resetPosition method of the estimator.
    def resetPose(self, pose: Pose2d):
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def robotOrientedDrive(self, vX, vY, vT):
        self.chassisSpeeds = ChassisSpeeds.discretize(
            ChassisSpeeds(vX, vY, vT), self.loopTime
        )

    def setChassisSpeeds(self, chassis_speeds):
        self.chassisSpeeds = ChassisSpeeds.discretize(chassis_speeds, self.loopTime)

    def setModuleStates(self, states):
        self.moduleStates = states

    def setStatesFromSpeeds(self):
        states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
        states = self.kinematics.desaturateWheelSpeeds(states, self.max_speed)
        self.moduleStates = states
