from enum import Enum
import math
from typing import Optional
from ntcore import NetworkTableInstance
from phoenix6 import StatusSignal
from phoenix6.configs.cancoder_configs import (
    CANcoderConfiguration,
    MagnetSensorConfigs,
)
from phoenix6.configs.talon_fx_configs import (
    TalonFXConfiguration,
    MotorOutputConfigs,
    CurrentLimitsConfigs,
)
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import (
    GravityTypeValue,
    InvertedValue,
    NeutralModeValue,
    SensorDirectionValue,
    StaticFeedforwardSignValue,
)
from wpimath.geometry import Rotation2d
from phoenix6.canbus import CANBus

import numpy as np
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations
from phoenix6 import unmanaged


# Motor output config for ClockWise Positive rotation and Brake neutral mode
MOTOR_OUTPUT_CWP_BRAKE = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
# Motor output config for Counter-ClockWise Positive rotation and Brake neutral mode
MOTOR_OUTPUT_CCWP_BRAKE = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
)
MOTOR_OUTPUT_CWP_COAST = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.COAST)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
MOTOR_OUTPUT_CCWP_COAST = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.COAST)
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
)

# Magnet sensor config for Counter-ClockWise Positive rotation with continuous wrap
MAGNET_CONFIG_CONTWRAP_CCWP = (
    MagnetSensorConfigs()
    .with_absolute_sensor_discontinuity_point(0.5)
    .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
)

MAX_FALCON_RPM = 6380  # maximum free speed of a Falcon 500
MAX_KRAKEN_X60_RPM = 6000  # estimated maximum free speed of a Kraken X60


def get_frog_talon_config() -> TalonFXConfiguration:
    """Returns a TalonFXConfiguration with FROG safe defaults (current limits)."""
    return TalonFXConfiguration().with_current_limits(
        CurrentLimitsConfigs()
        .with_supply_current_limit(40)
        .with_supply_current_limit_enable(True)
        .with_stator_current_limit(60)
        .with_stator_current_limit_enable(True)
    )


def get_frog_cancoder_config() -> CANcoderConfiguration:
    """Returns a CANcoderConfiguration with FROG defaults."""
    return (
        CANcoderConfiguration()
        .with_magnet_sensor(
            MagnetSensorConfigs()
            .with_absolute_sensor_discontinuity_point(0.5)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )
    )


class FROGTalonFX(TalonFX):
    """FROG custom TalonFX that takes parameters during instantiation."""

    class SignalProfile(Enum):
        """
        Frame-aligned frequency profiles for Phoenix 6.
        Tuple: (Primary, Secondary, Health) frame frequencies in Hz.
         - Primary Frame: Position, Velocity, Acceleration (used for closed-loop control)
         - Secondary Frame: Motor Voltage, Stator Current, Supply Voltage (crucial for current-limit tuning)
         - Health Frame: Device Temperature, Faults (important for monitoring and diagnostics)
        Tuning profile maxes out all frames for high-fidelity logging during PID tuning and
              System Identification (SysId).
        Production profiles use more conservative frequencies to optimize bus utilization.
        """

        # (Pos/Vel/Accel), (Volt/Curr), (Temp/Fault)
        SWERVE_DRIVE = (50, 50, 4)
        SWERVE_AZIMUTH = (50, 20, 4)
        FLYWHEEL = (50, 20, 4)  # 50Hz is fine for 'production'
        POSITION_MM = (50, 20, 4)
        FOLLOWER = (20, 20, 4)
        BASIC = (20, 20, 4)

        # --- TUNING MODE ---
        # Used for PID tuning and System Identification (SysId)
        # Maxes out the relevant frames for high-fidelity logging
        TUNING = (250, 100, 4)
        DISABLED = (4, 4, 4)

    def __init__(
        self,
        id: int,
        motor_config: TalonFXConfiguration = get_frog_talon_config(),
        canbus: str = "rio",
        motor_name: str = "",
        signal_profile: SignalProfile = SignalProfile.BASIC,
        motor_model: Optional[DCMotor] = None,
    ):
        """Creates a TalonFX motor object with applied configuration

        Args:
            id (int): The CAN ID of the motor.
            motor_config (TalonFXConfiguration): The configuration to apply to the motor. Defaults to a default configuration via get_frog_talon_config().
            canbus (str): The CAN bus string.
            motor_name (str): The name of the motor for logging purposes.
            signal_profile (SignalProfile): The signal profile.
            motor_model (Optional[DCMotor]): The motor model for simulation purposes.
        """
        super().__init__(device_id=id, canbus=CANBus(canbus))
        self.motor_name = motor_name if motor_name else f"TalonFX({id})"
        self.config = motor_config
        self.motor_model = motor_model
        self.configurator.apply(self.config)
        self.apply_usage_mode(signal_profile)

    def apply_usage_mode(self, signal_profile: SignalProfile):
        primary, secondary, health = signal_profile.value

        # Primary Frame: Motion data
        self.get_position().set_update_frequency(primary)
        self.get_velocity().set_update_frequency(primary)
        self.get_acceleration().set_update_frequency(primary)

        # Secondary Frame: Power data (Crucial for current-limit tuning)
        self.get_motor_voltage().set_update_frequency(secondary)
        self.get_stator_current().set_update_frequency(secondary)
        self.get_supply_voltage().set_update_frequency(secondary)

        # Health Frame: Thermal/Fault data
        self.get_device_temp().set_update_frequency(health)
        self.get_fault_field().set_update_frequency(health)

        # Disable/Slow everything else
        self.optimize_bus_utilization()

    def getMotorVoltage(self):
        return self.get_motor_voltage().value

    def is_stalled(
        self, current_threshold: float = 40.0, velocity_threshold: float = 0.1
    ) -> bool:
        """Returns True if the motor is likely stalled based on current and rotor velocity."""
        stator_current = self.get_stator_current().value
        velocity = abs(self.get_rotor_velocity().value)
        return stator_current > current_threshold and velocity < velocity_threshold

    def simulation_init(
        self,
        moi: float,
        motor_model: Optional[DCMotor] = None,
        gearing: Optional[float] = None,
        measurement_std_devs: Optional[list[float]] = None,
    ):
        """Initialize physics-based simulation for this motor.
        Automatically uses sensor_to_mechanism_ratio from the motor's 
        applied configuration if not overridden.

        Args:
            moi: Moment of Inertia (kg·m²) for the mechanism.
            motor_model: Optional DCMotor model. Defaults to self.motor_model or Falcon 500.
            gearing: Optional gear ratio override. Defaults to config.feedback.sensor_to_mechanism_ratio.
            measurement_std_devs: List of [pos_std, vel_std] for noise, defaults to [0.0, 0.0]
        """
        if motor_model is None:
            motor_model = self.motor_model or DCMotor.falcon500(1)

        # Extract gearing from the applied configuration
        if gearing is None:
            gearing = self.config.feedback.sensor_to_mechanism_ratio
        
        self._sim_gearing = gearing

        plant = LinearSystemId.DCMotorSystem(motor_model, moi, self._sim_gearing)

        if measurement_std_devs is None:
            measurement_std_devs = [0.0, 0.0]

        self.physim = DCMotorSim(plant, motor_model, np.array(measurement_std_devs))
        self.physim.setState(0.0, 0.0)

    def simulation_update(
        self,
        dt,
        battery_v,
        coupled_motors=None,
        max_velocity_rps=None,
        velocity_sign_multiplier=1,
    ):
        """Update simulation for this motor and optionally coupled motors.

        If physim is present, uses physics-based simulation.
        Otherwise, uses simple voltage-based simulation.

        Args:
            dt: Time step
            battery_v: Battery voltage
            coupled_motors: List of FROGTalonFX to set same sim state (for followers, physics only)
            max_velocity_rps: Max velocity at 12V (simple sim only)
            velocity_sign_multiplier: Direction multiplier (simple sim only)
        """
        if hasattr(self, "physim"):
            # Physics-based simulation
            unmanaged.feed_enable(0.100)
            self.sim_state.set_supply_voltage(battery_v)
            applied_v = self.sim_state.motor_voltage
            self.physim.setInputVoltage(applied_v)
            self.physim.update(dt)
            
            # Get output shaft state and convert to motor rotor state
            # pos_rot and vel_rps are in output units (e.g. meters or rotations)
            pos_output = self.physim.getAngularPositionRotations()
            vel_output = radiansToRotations(self.physim.getAngularVelocity())
            
            # Convert to rotor rotations/RPS by multiplying by gearing
            pos_rot = pos_output * self._sim_gearing
            vel_rps = vel_output * self._sim_gearing

            self.sim_state.set_raw_rotor_position(pos_rot)
            self.sim_state.set_rotor_velocity(vel_rps)
            if coupled_motors:
                for m in coupled_motors:
                    m.sim_state.set_supply_voltage(battery_v)
                    m.sim_state.set_raw_rotor_position(pos_rot)
                    m.sim_state.set_rotor_velocity(vel_rps)
        else:
            # Simple voltage-based simulation
            if max_velocity_rps is None:
                max_velocity_rps = 100.0  # Default RPS at 12V
            self.sim_state.set_supply_voltage(battery_v)
            applied_voltage = self.sim_state.motor_voltage
            target_velocity = (applied_voltage / 12.0) * max_velocity_rps  # type: ignore
            if not hasattr(self, "_sim_velocity"):
                self._sim_velocity = 0.0
            self._sim_velocity += 0.3 * (target_velocity - self._sim_velocity)
            directed_velocity = velocity_sign_multiplier * self._sim_velocity
            self.sim_state.set_rotor_velocity(directed_velocity)
            position_change = directed_velocity * dt
            self.sim_state.add_rotor_position(position_change)


class FROGPigeonGyro(Pigeon2):
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self, can_id: int):
        super().__init__(can_id)
        self.reset()
        self.optimize_bus_utilization()
        self.get_yaw().set_update_frequency(100)

    def getAngleCCW(self) -> float:
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return self.getYaw()

    def getYaw(self) -> float:
        return self.get_yaw().value

    def getRoll(self) -> float:
        return self.get_roll().value

    def getPitch(self) -> float:
        return self.get_pitch().value

    def getDegreesPerSecCCW(self) -> float:
        return self.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self) -> float:
        return math.radians(self.getDegreesPerSecCCW())

    def setAngleAdjustment(self, angle):
        self.set_yaw(angle)


class FROGCanCoder(CANcoder):
    def __init__(
        self,
        id: int,
        config: CANcoderConfiguration = get_frog_cancoder_config(),
        canbus: str = "rio",
    ):
        super().__init__(id, canbus)
        self.configurator.apply(config)
        self.optimize_bus_utilization()
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)
