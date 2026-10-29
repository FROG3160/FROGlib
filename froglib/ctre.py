from dataclasses import dataclass, field
from enum import Enum, auto
import math
import time
from typing import Any, Callable, List, Optional, Tuple, Union

# ------------------------------------------------------------------------------
# Imports with graceful fallbacks for testing/headless environments
# ------------------------------------------------------------------------------

try:
    from ntcore import NetworkTableInstance
except ImportError:
    class _DummyPublisher:
        def set(self, val: Any):
            pass

    class _DummyTopic:
        def publish(self):
            return _DummyPublisher()

    class _DummyNT:
        def getStringTopic(self, _):
            return _DummyTopic()

        def getFloatTopic(self, _):
            return _DummyTopic()

    class NetworkTableInstance:  # type: ignore
        @staticmethod
        def getDefault():
            return _DummyNT()

try:
    from phoenix6 import StatusSignal, SignalLogger, unmanaged
    from phoenix6.canbus import CANBus
    from phoenix6.configs.cancoder_configs import (
        CANcoderConfiguration,
        MagnetSensorConfigs,
    )
    from phoenix6.configs.config_groups import (
        CurrentLimitsConfigs,
        FeedbackConfigs,
        Slot0Configs,
        Slot1Configs,
        SoftwareLimitSwitchConfigs,
    )
    from phoenix6.configs.talon_fx_configs import (
        FeedbackSensorSourceValue,
        MotorOutputConfigs,
        TalonFXConfiguration,
    )
    from phoenix6.controls import PositionVoltage, VoltageOut
    from phoenix6.hardware.cancoder import CANcoder
    from phoenix6.hardware.pigeon2 import Pigeon2
    from phoenix6.hardware.talon_fx import TalonFX
    from phoenix6.signals.spn_enums import (
        GravityTypeValue,
        InvertedValue,
        NeutralModeValue,
        SensorDirectionValue,
        StaticFeedforwardSignValue,
    )
except ImportError:
    StatusSignal = object  # type: ignore
    SignalLogger = None  # type: ignore

    class _DummyUnmanaged:
        @staticmethod
        def feed_enable(_):
            pass

    unmanaged = _DummyUnmanaged()  # type: ignore

    class CANBus:  # type: ignore
        def __init__(self, name: str = "rio"):
            self.name = name

    class CANcoderConfiguration:  # type: ignore
        def with_magnet_sensor(self, _):
            return self

    class MagnetSensorConfigs:  # type: ignore
        def with_sensor_direction(self, _):
            return self

        def with_absolute_sensor_range(self, _):
            return self

    class Slot0Configs:  # type: ignore
        def __init__(self):
            self.k_s = 0.0
            self.k_v = 0.0
            self.k_a = 0.0
            self.k_g = 0.0
            self.k_p = 0.0
            self.k_d = 0.0

    class Slot1Configs(Slot0Configs):  # type: ignore
        pass

    class CurrentLimitsConfigs:  # type: ignore
        def __init__(self):
            self.stator_current_limit_enable = False
            self.stator_current_limit = 0.0

    class SoftwareLimitSwitchConfigs:  # type: ignore
        pass

    class FeedbackConfigs:  # type: ignore
        def __init__(self):
            self.sensor_to_mechanism_ratio = 1.0
            self.rotor_to_sensor_ratio = 1.0

    class MotorOutputConfigs:  # type: ignore
        def with_neutral_mode(self, _):
            return self

        def with_inverted(self, _):
            return self

    class TalonFXConfiguration:  # type: ignore
        def __init__(self):
            self.feedback = FeedbackConfigs()
            self.slot0 = Slot0Configs()
            self.slot1 = Slot1Configs()
            self.current_limits = CurrentLimitsConfigs()

    class FeedbackSensorSourceValue:  # type: ignore
        ROTOR_SENSOR = 0

    class VoltageOut:  # type: ignore
        def __init__(self, output: float):
            self.output = output

    class PositionVoltage:  # type: ignore
        def __init__(self, position: float):
            self.position = position

    class CANcoder:  # type: ignore
        def __init__(self, id: int, canbus: Any = "rio"):
            self.id = id
            self.configurator = self

        def apply(self, _):
            pass

        def optimize_bus_utilization(self):
            pass

    class Pigeon2:  # type: ignore
        def __init__(self, id: int):
            self.id = id

        def reset(self):
            pass

        def optimize_bus_utilization(self):
            pass

        def get_yaw(self):
            return self

        def set_update_frequency(self, _):
            pass

    class _DummySignal:
        def __init__(self, value: float = 0.0):
            self.value = value

        def set_update_frequency(self, _):
            pass

    class TalonFX:  # type: ignore
        def __init__(self, device_id: int, canbus: Any = "rio"):
            self.device_id = device_id
            self.configurator = self

        def apply(self, _):
            pass

        def optimize_bus_utilization(self):
            pass

        def get_position(self):
            return _DummySignal(0.0)

        def get_velocity(self):
            return _DummySignal(0.0)

        def get_acceleration(self):
            return _DummySignal(0.0)

        def get_motor_voltage(self):
            return _DummySignal(0.0)

        def get_stator_current(self):
            return _DummySignal(0.0)

        def get_supply_voltage(self):
            return _DummySignal(12.0)

        def get_device_temp(self):
            return _DummySignal(25.0)

        def get_fault_field(self):
            return _DummySignal(0.0)

        def get_rotor_velocity(self):
            return _DummySignal(0.0)

        def set_control(self, _):
            pass

    class GravityTypeValue:  # type: ignore
        pass

    class InvertedValue:  # type: ignore
        CLOCKWISE_POSITIVE = 0
        COUNTER_CLOCKWISE_POSITIVE = 1

    class NeutralModeValue:  # type: ignore
        COAST = 0
        BRAKE = 1

    class SensorDirectionValue:  # type: ignore
        COUNTER_CLOCKWISE_POSITIVE = 1

    class StaticFeedforwardSignValue:  # type: ignore
        pass

try:
    from wpimath.geometry import Rotation2d
    from wpilib.simulation import DCMotorSim
    from wpimath.system.plant import DCMotor, LinearSystemId
    from wpimath.units import radiansToRotations
except ImportError:
    class Rotation2d:  # type: ignore
        def __init__(self, value: float = 0.0):
            self.value = value

        @staticmethod
        def fromDegrees(deg: float):
            return Rotation2d(math.radians(deg))

    class DCMotor:  # type: ignore
        @staticmethod
        def falcon500(_=1):
            return DCMotor()

    class LinearSystemId:  # type: ignore
        @staticmethod
        def DCMotorSystem(*_):
            return object()

    class DCMotorSim:  # type: ignore
        def __init__(self, *_):
            pass

        def setState(self, *_):
            pass

    def radiansToRotations(rad: float) -> float:
        return rad / (2.0 * math.pi)

try:
    from commands2 import Command
    from commands2.sysid import SysIdRoutine
    from wpimath.sysid import SysIdRoutineLog
    from wpimath.units import volts, seconds, volts_per_second
except ImportError:
    Command = object  # type: ignore
    SysIdRoutine = None  # type: ignore
    SysIdRoutineLog = None  # type: ignore

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore


# ------------------------------------------------------------------------------
# Standard Motor & Sensor Configurations
# ------------------------------------------------------------------------------

MOTOR_OUTPUT_CWP_BRAKE = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
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

MAGNET_SENSOR_CCWP = (
    MagnetSensorConfigs()
    .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
    .with_absolute_sensor_range(0)
)


def get_frog_talon_config() -> TalonFXConfiguration:
    return TalonFXConfiguration()


def get_frog_cancoder_config() -> CANcoderConfiguration:
    return CANcoderConfiguration().with_magnet_sensor(MAGNET_SENSOR_CCWP)


# ------------------------------------------------------------------------------
# Mathematical Utilities for Characterization
# ------------------------------------------------------------------------------


def linear_regression(x_data: List[float], y_data: List[float]) -> Tuple[float, float, float]:
    """Computes ordinary least squares linear regression: y = slope * x + intercept.

    Args:
        x_data: Independent variable samples (e.g., velocity in RPS).
        y_data: Dependent variable samples (e.g., effective voltage in Volts).

    Returns:
        Tuple of (slope, intercept, r_squared).
    """
    n = len(x_data)
    if n < 2:
        return 0.0, 0.0, 0.0

    x_mean = sum(x_data) / n
    y_mean = sum(y_data) / n

    ss_xx = sum((x - x_mean) ** 2 for x in x_data)
    ss_xy = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_data, y_data))
    ss_yy = sum((y - y_mean) ** 2 for y in y_data)

    if ss_xx == 0.0:
        return 0.0, float(y_mean), 0.0

    slope = float(ss_xy / ss_xx)
    intercept = float(y_mean - slope * x_mean)

    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(x_data, y_data))
    r_squared = 1.0 - float(ss_res / ss_yy) if ss_yy > 0.0 else 1.0

    return slope, intercept, max(0.0, min(1.0, r_squared))


def calculate_feedback_gains(
    k_v: float,
    k_a: float,
    target_bandwidth_rad_s: float = 25.0,
    damping_ratio: float = 1.0,
) -> Tuple[float, float]:
    """Calculates recommended proportional (kP) and derivative (kD) feedback gains
    using second-order continuous pole placement for critically damped response.

    Matching G(s) = 1 / (k_a * s + k_v) with PD controller to s^2 + 2*zeta*omega_n*s + omega_n^2:
        k_p = k_a * omega_n^2
        k_d = 2 * zeta * omega_n * k_a - k_v

    Args:
        k_v: Velocity feedforward gain in Volts / RPS.
        k_a: Acceleration feedforward gain in Volts / (RPS/s).
        target_bandwidth_rad_s: Desired closed-loop natural frequency omega_n in rad/s (default 25.0).
        damping_ratio: Damping ratio zeta (default 1.0 for critical damping).

    Returns:
        Tuple of (k_p, k_d) in units compatible with Phoenix 6 Slot0Configs.
    """
    omega_n = max(0.1, target_bandwidth_rad_s)
    zeta = max(0.1, damping_ratio)
    k_a_safe = max(1e-4, k_a)

    k_p = k_a_safe * (omega_n**2)
    k_d = (2.0 * zeta * omega_n * k_a_safe) - k_v
    k_d = max(0.0, k_d)

    return float(k_p), float(k_d)


# ------------------------------------------------------------------------------
# Characterization Dataclasses
# ------------------------------------------------------------------------------


@dataclass
class CharacterizationConfig:
    """Configuration parameters for automated TalonFX characterization and gain calculation."""

    mechanism_name: str = "TalonFX_Mechanism"

    # Mechanism Travel Constraints (for limited-travel systems like arms/elevators)
    is_bounded: bool = False
    min_test_pos: float = float("-inf")  # Minimum safe position in mechanism rotations
    max_test_pos: float = float("inf")  # Maximum safe position in mechanism rotations
    safe_home_pos: float = 0.0  # Safe reset position between test stages
    soft_limit_buffer: float = 0.1  # Margin from travel limits to avoid boundary trip

    # Safety Limits
    max_voltage: float = 10.0  # Maximum voltage allowed during testing (V)
    stator_current_limit_amps: float = 20.0  # Temporary stator current clamp during test (A)
    step_timeout_s: float = 2.5  # Maximum timeout per test step before auto-abort
    stall_detection_enabled: bool = True
    stall_current_threshold: float = 18.0  # Current threshold for stall detection (A)
    stall_velocity_threshold: float = 0.05  # Velocity threshold for stall detection (RPS)
    stall_duration_threshold_s: float = 0.15  # Duration of stall before triggering emergency abort

    # Test Parameters
    ramp_rate_v_per_s: float = 0.5  # Voltage ramp rate for kS breakaway test (V/s)
    breakaway_threshold_rps: float = 0.1  # Velocity threshold to register breakaway motion

    kv_test_voltages: List[float] = field(
        default_factory=lambda: [2.0, 4.0, 6.0, 8.0, 10.0]
    )
    kv_settle_time_s: float = 0.35  # Settling time for steady-state velocity measurement

    ka_step_voltage: float = 6.0  # Voltage step for acceleration/inertia test (V)
    ka_step_duration_s: float = 0.15  # Duration of acceleration pulse (short to limit travel)

    # Feedback Synthesis (Pole Placement)
    target_bandwidth_rad_s: float = 25.0  # Closed-loop natural frequency omega_n (rad/s)
    damping_ratio: float = 1.0  # Critical damping zeta = 1.0

    # Hot-Apply and Slot Configuration
    auto_apply: bool = False  # Whether to apply gains directly to Slot 0/1 on completion
    target_slot: int = 0  # Which slot to configure (0 or 1)
    distance_per_rotation_meters: Optional[float] = None  # Optional conversion for linear mechanisms


@dataclass
class TuningResult:
    """Structured results from automated TalonFX characterization."""

    mechanism_name: str
    k_s: float  # Volts
    k_v: float  # Volts / RPS
    k_a: float  # Volts / (RPS/s)
    k_g: Optional[float] = None  # Volts (gravity feedforward for arms/elevators)
    k_p: float = 0.0  # Volts / rotation (position) or Volts / RPS (velocity)
    k_d: float = 0.0  # Volts / RPS (position) or Volts / (RPS/s) (velocity)
    r_squared: float = 1.0  # Linearity score of kV regression fit (0.0 to 1.0)
    gearing: float = 1.0  # Sensor-to-mechanism ratio applied
    is_mechanism_units: bool = True
    timestamp: float = field(default_factory=time.time)

    def to_slot0_configs(self) -> Slot0Configs:
        """Exports the tuning results into a Phoenix 6 Slot0Configs object."""
        configs = Slot0Configs()
        configs.k_s = self.k_s
        configs.k_v = self.k_v
        configs.k_a = self.k_a
        if self.k_g is not None:
            configs.k_g = self.k_g
        configs.k_p = self.k_p
        configs.k_d = self.k_d
        return configs

    def to_slot1_configs(self) -> Slot1Configs:
        """Exports the tuning results into a Phoenix 6 Slot1Configs object."""
        configs = Slot1Configs()
        configs.k_s = self.k_s
        configs.k_v = self.k_v
        configs.k_a = self.k_a
        if self.k_g is not None:
            configs.k_g = self.k_g
        configs.k_p = self.k_p
        configs.k_d = self.k_d
        return configs

    def to_python_code(self) -> str:
        """Returns executable Python snippet to apply these gains in robot configuration."""
        slot_name = "slot0"
        kg_line = f"    cfg.{slot_name}.k_g = {self.k_g:.4f}\n" if self.k_g is not None else ""
        return (
            f"# Generated gains for {self.mechanism_name} (R² = {self.r_squared:.4f})\n"
            f"def configure_gains(cfg: TalonFXConfiguration):\n"
            f"    cfg.{slot_name}.k_s = {self.k_s:.4f}\n"
            f"    cfg.{slot_name}.k_v = {self.k_v:.4f}\n"
            f"    cfg.{slot_name}.k_a = {self.k_a:.4f}\n"
            f"{kg_line}"
            f"    cfg.{slot_name}.k_p = {self.k_p:.4f}\n"
            f"    cfg.{slot_name}.k_d = {self.k_d:.4f}\n"
        )

    def format_report(self) -> str:
        """Returns a formatted multi-line summary of the characterization results."""
        unit_str = "Mechanism" if self.is_mechanism_units else "Rotor"
        kg_str = f"kG (Gravity)   : {self.k_g:.4f} V\n" if self.k_g is not None else ""
        return (
            f"\n{'=' * 60}\n"
            f"  CHARACTERIZATION REPORT: {self.mechanism_name}\n"
            f"{'=' * 60}\n"
            f"Coordinate Frame: {unit_str} Units (Gearing = {self.gearing:.2f}:1)\n"
            f"R² Fit Score    : {self.r_squared:.4f} ({'Optimal' if self.r_squared >= 0.98 else 'Review Data'})\n"
            f"kS (Static)     : {self.k_s:.4f} V\n"
            f"kV (Velocity)   : {self.k_v:.4f} V / rps\n"
            f"kA (Accel)      : {self.k_a:.4f} V / (rps/s)\n"
            f"{kg_str}"
            f"Recommended kP  : {self.k_p:.4f} V / rotation\n"
            f"Recommended kD  : {self.k_d:.4f} V / rps\n"
            f"{'=' * 60}\n"
        )


# ------------------------------------------------------------------------------
# Autonomous Characterizer & SysId Helper
# ------------------------------------------------------------------------------


class TalonFXCharacterizer:
    """Automated testing routine for determining TalonFX firmware gains.

    Supports both continuous mechanisms (flywheels, swerve drive) and bounded
    mechanisms (elevators, arms, wrists). Automatically manages signal frequencies,
    temporary current limits, stall monitoring, and gear ratio validation.
    """

    class State(Enum):
        IDLE = auto()
        STARTUP = auto()
        KS_FORWARD = auto()
        RETURN_HOME_1 = auto()
        KS_REVERSE = auto()
        RETURN_HOME_2 = auto()
        KV_SWEEP = auto()
        RETURN_HOME_3 = auto()
        KA_STEP = auto()
        RETURN_HOME_4 = auto()
        COMPUTE_AND_FINISH = auto()
        ABORTED = auto()

    def __init__(self, motor: Any, config: Optional[CharacterizationConfig] = None):
        """Initializes the characterizer for a given motor and configuration."""
        self.motor = motor
        self.config = config or CharacterizationConfig()
        self.state = self.State.IDLE

        # State tracking
        self._state_start_time = 0.0
        self._current_voltage = 0.0
        self._ks_fwd = 0.0
        self._ks_rev = 0.0
        self._ks_break_vel = 0.0
        self._ks_break_accel = 0.0
        self._kv_samples_rps: List[float] = []
        self._kv_samples_volts: List[float] = []
        self._kv_index = 0
        self._ka_accel = 0.0
        self._ka_vel = 0.0
        self._abort_reason = ""
        self._result: Optional[TuningResult] = None

        self._original_stator_limit = 40.0
        self._stall_start_time: Optional[float] = None

        # NetworkTable publisher for telemetry
        nt_table = f"FROGLib/Tuning/{self.config.mechanism_name}"
        self._state_pub = NetworkTableInstance.getDefault().getStringTopic(f"{nt_table}/State").publish()
        self._volts_pub = NetworkTableInstance.getDefault().getFloatTopic(f"{nt_table}/AppliedVolts").publish()
        self._vel_pub = NetworkTableInstance.getDefault().getFloatTopic(f"{nt_table}/VelocityRPS").publish()

    def validate_gearing(self, measured_kv: float, motor_free_speed_rps: float = 100.0) -> Tuple[bool, str]:
        """Validates that measured kV matches expected theoretical gearing."""
        gearing = self.get_active_gearing()
        theoretical_kv = (12.0 / motor_free_speed_rps) * gearing

        if theoretical_kv <= 0.0:
            return True, "No theoretical baseline available."

        discrepancy = abs(measured_kv - theoretical_kv) / theoretical_kv
        if discrepancy > 0.40:
            msg = (
                f"[WARNING] Measured kV ({measured_kv:.4f} V/rps) deviates by {discrepancy * 100:.1f}% "
                f"from theoretical expectation ({theoretical_kv:.4f} V/rps for {gearing:.2f}:1 reduction). "
                f"Verify that sensor_to_mechanism_ratio is configured correctly!"
            )
            return False, msg
        return True, f"Measured kV closely aligns with theoretical expectation ({theoretical_kv:.4f} V/rps)."

    def get_active_gearing(self) -> float:
        """Retrieves the effective gear reduction from the motor's feedback configuration."""
        try:
            gearing = self.motor.config.feedback.sensor_to_mechanism_ratio
            return gearing if gearing > 0.0 else 1.0
        except AttributeError:
            return 1.0

    def check_safety_guards(self) -> bool:
        """Evaluates active position boundaries, stall detection, and timeout safety guards."""
        current_pos = self.motor.get_position().value
        current_vel = abs(self.motor.get_velocity().value)
        stator_curr = abs(self.motor.get_stator_current().value)
        now = time.monotonic()

        # 1. Bounded Travel Protection
        if self.config.is_bounded:
            effective_min = self.config.min_test_pos + self.config.soft_limit_buffer
            effective_max = self.config.max_test_pos - self.config.soft_limit_buffer

            if self._current_voltage > 0.0 and current_pos >= effective_max:
                self.emergency_abort(f"Forward travel limit reached ({current_pos:.2f} >= {effective_max:.2f})")
                return False
            if self._current_voltage < 0.0 and current_pos <= effective_min:
                self.emergency_abort(f"Reverse travel limit reached ({current_pos:.2f} <= {effective_min:.2f})")
                return False

        # 2. Stall / Jam Detection
        if self.config.stall_detection_enabled:
            is_stalling = (
                stator_curr > self.config.stall_current_threshold
                and current_vel < self.config.stall_velocity_threshold
                and abs(self._current_voltage) > 1.0
            )

            if is_stalling:
                if self._stall_start_time is None:
                    self._stall_start_time = now
                elif now - self._stall_start_time >= self.config.stall_duration_threshold_s:
                    self.emergency_abort(
                        f"Motor stall detected! Stator current {stator_curr:.1f}A at velocity {current_vel:.2f} RPS"
                    )
                    return False
            else:
                self._stall_start_time = None

        # 3. Step Execution Timeout
        if now - self._state_start_time > self.config.step_timeout_s:
            self.emergency_abort(f"Test step {self.state.name} exceeded maximum timeout of {self.config.step_timeout_s}s")
            return False

        return True

    def emergency_abort(self, reason: str):
        """Immediately zeroes motor output and halts the testing sequence."""
        self._abort_reason = reason
        self.state = self.State.ABORTED
        self._set_voltage(0.0)
        self.teardown()
        print(f"\n[EMERGENCY ABORT] {self.config.mechanism_name}: {reason}\n")

    def setup(self):
        """Pre-flight setup: switches to 250Hz TUNING profile and clamps current limits."""
        if hasattr(self.motor, "apply_usage_mode"):
            tuning_prof = getattr(FROGTalonFX.SignalProfile, "TUNING", None)
            if tuning_prof is not None:
                self.motor.apply_usage_mode(tuning_prof)

        curr_config = CurrentLimitsConfigs()
        curr_config.stator_current_limit_enable = True
        curr_config.stator_current_limit = self.config.stator_current_limit_amps
        if hasattr(self.motor, "configurator"):
            self.motor.configurator.apply(curr_config)

        self._state_start_time = time.monotonic()
        self._current_voltage = 0.0
        self._kv_samples_rps.clear()
        self._kv_samples_volts.clear()
        self._kv_index = 0
        self._abort_reason = ""
        self._result = None
        self.state = self.State.KS_FORWARD

    def teardown(self):
        """Restores motor to safe zero output and default production profile."""
        self._set_voltage(0.0)

        if hasattr(self.motor, "apply_usage_mode"):
            basic_prof = getattr(FROGTalonFX.SignalProfile, "BASIC", None)
            if basic_prof is not None:
                self.motor.apply_usage_mode(basic_prof)

        curr_config = CurrentLimitsConfigs()
        curr_config.stator_current_limit_enable = True
        curr_config.stator_current_limit = self._original_stator_limit
        if hasattr(self.motor, "configurator"):
            self.motor.configurator.apply(curr_config)

    def _set_voltage(self, volts: float):
        clamped_volts = max(-self.config.max_voltage, min(self.config.max_voltage, volts))
        self._current_voltage = clamped_volts
        if hasattr(self.motor, "set_control"):
            self.motor.set_control(VoltageOut(clamped_volts))

        self._state_pub.set(self.state.name)
        self._volts_pub.set(clamped_volts)
        try:
            self._vel_pub.set(self.motor.get_velocity().value)
        except Exception:
            pass

    def periodic(self) -> bool:
        """Executes one tick of the characterization state machine.

        Returns:
            True if testing has finished (successfully or aborted), False if still running.
        """
        if self.state in (self.State.IDLE, self.State.COMPUTE_AND_FINISH, self.State.ABORTED):
            return True

        if not self.check_safety_guards():
            return True

        now = time.monotonic()
        dt = max(0.001, now - self._state_start_time)
        velocity = self.motor.get_velocity().value

        if self.state == self.State.KS_FORWARD:
            if velocity >= self.config.breakaway_threshold_rps and self._current_voltage > 0.0:
                self._ks_fwd = self._current_voltage
                self._ks_break_vel = abs(velocity)
                self._ks_break_accel = abs(getattr(self.motor.get_acceleration(), "value", 0.0))
                self._transition_to(self.State.RETURN_HOME_1)
            else:
                target_v = dt * self.config.ramp_rate_v_per_s
                self._set_voltage(target_v)
                if velocity >= self.config.breakaway_threshold_rps:
                    self._ks_fwd = self._current_voltage
                    self._ks_break_vel = abs(velocity)
                    self._ks_break_accel = abs(getattr(self.motor.get_acceleration(), "value", 0.0))
                    self._transition_to(self.State.RETURN_HOME_1)

        elif self.state == self.State.RETURN_HOME_1:
            if self._handle_return_home():
                self._transition_to(self.State.KS_REVERSE)

        elif self.state == self.State.KS_REVERSE:
            if velocity <= -self.config.breakaway_threshold_rps and self._current_voltage < 0.0:
                self._ks_rev = abs(self._current_voltage)
                self._ks_break_vel = 0.5 * (self._ks_break_vel + abs(velocity))
                self._ks_break_accel = 0.5 * (
                    self._ks_break_accel + abs(getattr(self.motor.get_acceleration(), "value", 0.0))
                )
                self._transition_to(self.State.RETURN_HOME_2)
            else:
                target_v = -(dt * self.config.ramp_rate_v_per_s)
                self._set_voltage(target_v)
                if velocity <= -self.config.breakaway_threshold_rps:
                    self._ks_rev = abs(self._current_voltage)
                    self._ks_break_vel = 0.5 * (self._ks_break_vel + abs(velocity))
                    self._ks_break_accel = 0.5 * (
                        self._ks_break_accel + abs(getattr(self.motor.get_acceleration(), "value", 0.0))
                    )
                    self._transition_to(self.State.RETURN_HOME_2)

        elif self.state == self.State.RETURN_HOME_2:
            if self._handle_return_home():
                self._kv_index = 0
                self._transition_to(self.State.KV_SWEEP)

        elif self.state == self.State.KV_SWEEP:
            if self._kv_index < len(self.config.kv_test_voltages):
                v_target = self.config.kv_test_voltages[self._kv_index]
                self._set_voltage(v_target)
                if dt >= self.config.kv_settle_time_s:
                    self._kv_samples_volts.append(v_target)
                    self._kv_samples_rps.append(velocity)
                    self._kv_index += 1
                    self._state_start_time = now
            else:
                self._transition_to(self.State.RETURN_HOME_3)

        elif self.state == self.State.RETURN_HOME_3:
            if self._handle_return_home():
                self._transition_to(self.State.KA_STEP)

        elif self.state == self.State.KA_STEP:
            self._set_voltage(self.config.ka_step_voltage)
            if dt >= self.config.ka_step_duration_s:
                accel = getattr(self.motor.get_acceleration(), "value", 0.0)
                if abs(accel) < 0.1 and dt > 0:
                    accel = velocity / dt
                    self._ka_vel = 0.5 * abs(velocity)
                else:
                    self._ka_vel = abs(velocity)
                self._ka_accel = abs(accel)
                self._set_voltage(0.0)
                self._transition_to(self.State.RETURN_HOME_4)

        elif self.state == self.State.RETURN_HOME_4:
            if self._handle_return_home():
                self._compute_results()
                self.teardown()
                self.state = self.State.COMPUTE_AND_FINISH
                return True

        return False

    def _transition_to(self, new_state: State):
        self._set_voltage(0.0)
        self.state = new_state
        self._state_start_time = time.monotonic()
        self._stall_start_time = None

    def _handle_return_home(self) -> bool:
        if not self.config.is_bounded:
            self._set_voltage(0.0)
            return abs(self.motor.get_velocity().value) < 0.05

        current_pos = self.motor.get_position().value
        pos_error = self.config.safe_home_pos - current_pos

        if abs(pos_error) < 0.1:
            self._set_voltage(0.0)
            return abs(self.motor.get_velocity().value) < 0.05

        return_voltage = math.copysign(min(2.0, max(0.5, abs(pos_error) * 2.0)), pos_error)
        self._set_voltage(return_voltage)
        return False

    def _compute_results(self):
        raw_ks = (self._ks_fwd + self._ks_rev) / 2.0 if (self._ks_fwd > 0 and self._ks_rev > 0) else self._ks_fwd
        k_v, _, r_squared = linear_regression(self._kv_samples_rps, self._kv_samples_volts)

        # Compensate breakaway voltage for back-EMF and rotor inertia at the detection threshold
        k_s = max(0.0, raw_ks - (k_v * self._ks_break_vel))
        accel_volts = max(0.0, self.config.ka_step_voltage - k_s - (k_v * self._ka_vel))
        k_a = accel_volts / max(0.1, self._ka_accel) if self._ka_accel > 0 and accel_volts > 0 else 0.005

        if self._ks_break_accel > 0.0 and raw_ks > 0.0:
            k_s = max(0.0, raw_ks - (k_v * self._ks_break_vel) - (k_a * self._ks_break_accel))
            accel_volts = max(0.0, self.config.ka_step_voltage - k_s - (k_v * self._ka_vel))
            if self._ka_accel > 0 and accel_volts > 0:
                k_a = accel_volts / max(0.1, self._ka_accel)

        k_p, k_d = calculate_feedback_gains(
            k_v,
            k_a,
            target_bandwidth_rad_s=self.config.target_bandwidth_rad_s,
            damping_ratio=self.config.damping_ratio,
        )

        gearing = self.get_active_gearing()

        self._result = TuningResult(
            mechanism_name=self.config.mechanism_name,
            k_s=float(k_s),
            k_v=float(k_v),
            k_a=float(k_a),
            k_p=float(k_p),
            k_d=float(k_d),
            r_squared=float(r_squared),
            gearing=float(gearing),
            is_mechanism_units=True,
        )

        is_valid, validation_msg = self.validate_gearing(k_v)
        print(self._result.format_report())
        if not is_valid:
            print(validation_msg)

        if self.config.auto_apply and hasattr(self.motor, "configurator"):
            slot_config = (
                self._result.to_slot1_configs()
                if self.config.target_slot == 1
                else self._result.to_slot0_configs()
            )
            self.motor.configurator.apply(slot_config)
            print(
                f"[HOT APPLY] Successfully applied gains to Slot {self.config.target_slot} on {self.config.mechanism_name}."
            )

    def get_result(self) -> Optional[TuningResult]:
        return self._result

    def create_characterization_command(self) -> Any:
        """Creates a runnable WPILib Command that executes the characterization routine."""
        try:
            from commands2 import FunctionalCommand

            return FunctionalCommand(
                onInit=self.setup,
                onExecute=lambda: self.periodic(),
                onEnd=lambda interrupted: self.teardown() if interrupted else None,
                isFinished=lambda: self.state in (self.State.COMPUTE_AND_FINISH, self.State.ABORTED),
            )
        except ImportError:
            return None


class FROGSysIdHelper:
    """Helper class integrating WPILib's SysIdRoutine with CTRE Phoenix 6 SignalLogger."""

    def __init__(
        self,
        motor: Any,
        mechanism_name: str = "Mechanism",
        ramp_rate_v_per_s: float = 1.0,
        step_voltage: float = 7.0,
        timeout_s: float = 10.0,
        is_bounded: bool = False,
        min_pos: float = float("-inf"),
        max_pos: float = float("inf"),
    ):
        self.motor = motor
        self.mechanism_name = mechanism_name
        self.is_bounded = is_bounded
        self.min_pos = min_pos
        self.max_pos = max_pos

        if SysIdRoutine is None:
            self._routine = None
            return

        routine_config = SysIdRoutine.Config(
            rampRate=volts_per_second(ramp_rate_v_per_s),
            stepVoltage=volts(step_voltage),
            timeout=seconds(timeout_s),
            recordState=self._record_state,
        )

        mechanism = SysIdRoutine.Mechanism(
            drive=self._drive_voltage,
            log=None,
        )

        self._routine = SysIdRoutine(routine_config, mechanism)

    def _drive_voltage(self, voltage_val: float):
        current_pos = self.motor.get_position().value
        if self.is_bounded:
            if (voltage_val > 0.0 and current_pos >= self.max_pos) or (
                voltage_val < 0.0 and current_pos <= self.min_pos
            ):
                self.motor.set_control(VoltageOut(0.0))
                return
        self.motor.set_control(VoltageOut(voltage_val))

    def _record_state(self, state: "SysIdRoutineLog.State"):
        if SignalLogger is not None:
            SignalLogger.write_string(f"SysIdRoutine_State_{self.mechanism_name}", str(state))

    def quasistatic_forward(self) -> Any:
        if self._routine is None:
            return None
        return self._wrap_sysid_command(self._routine.quasistatic(SysIdRoutine.Direction.kForward))

    def quasistatic_reverse(self) -> Any:
        if self._routine is None:
            return None
        return self._wrap_sysid_command(self._routine.quasistatic(SysIdRoutine.Direction.kReverse))

    def dynamic_forward(self) -> Any:
        if self._routine is None:
            return None
        return self._wrap_sysid_command(self._routine.dynamic(SysIdRoutine.Direction.kForward))

    def dynamic_reverse(self) -> Any:
        if self._routine is None:
            return None
        return self._wrap_sysid_command(self._routine.dynamic(SysIdRoutine.Direction.kReverse))

    def _wrap_sysid_command(self, cmd: Any) -> Any:
        try:
            from commands2 import FunctionalCommand

            return FunctionalCommand(
                onInit=lambda: self._on_start(),
                onExecute=cmd.execute,
                onEnd=lambda interrupted: self._on_end(interrupted, cmd),
                isFinished=cmd.isFinished,
            )
        except ImportError:
            return None

    def _on_start(self):
        if hasattr(self.motor, "apply_usage_mode"):
            prof = getattr(FROGTalonFX.SignalProfile, "TUNING", None)
            if prof is not None:
                self.motor.apply_usage_mode(prof)
        if SignalLogger is not None:
            SignalLogger.start()

    def _on_end(self, interrupted: bool, cmd: Any):
        if hasattr(cmd, "end"):
            cmd.end(interrupted)
        if hasattr(self.motor, "set_control"):
            self.motor.set_control(VoltageOut(0.0))
        if hasattr(self.motor, "apply_usage_mode"):
            prof = getattr(FROGTalonFX.SignalProfile, "BASIC", None)
            if prof is not None:
                self.motor.apply_usage_mode(prof)
        if SignalLogger is not None:
            SignalLogger.stop()


# ------------------------------------------------------------------------------
# FROGTalonFX Hardware Abstraction
# ------------------------------------------------------------------------------


class FROGTalonFX(TalonFX):
    """FROG custom TalonFX that takes parameters during instantiation."""

    class SignalProfile(Enum):
        """Frame-aligned frequency profiles for Phoenix 6.
        Tuple: (Primary, Secondary, Health) frame frequencies in Hz.
        """

        SWERVE_DRIVE = (50, 50, 4)
        SWERVE_AZIMUTH = (50, 20, 4)
        FLYWHEEL = (50, 20, 4)
        POSITION_MM = (50, 20, 4)
        FOLLOWER = (20, 20, 4)
        BASIC = (20, 20, 4)

        # --- TUNING MODE ---
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

        # Secondary Frame: Power data
        self.get_motor_voltage().set_update_frequency(secondary)
        self.get_stator_current().set_update_frequency(secondary)
        self.get_supply_voltage().set_update_frequency(secondary)

        # Health Frame: Thermal/Fault data
        self.get_device_temp().set_update_frequency(health)
        self.get_fault_field().set_update_frequency(health)

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

    def get_characterizer(
        self, config: Optional[CharacterizationConfig] = None
    ) -> TalonFXCharacterizer:
        """Creates an autonomous TalonFXCharacterizer instance for this motor."""
        return TalonFXCharacterizer(self, config)

    def create_characterization_command(
        self, config: Optional[CharacterizationConfig] = None
    ) -> Any:
        """Creates a runnable WPILib Command that executes autonomous characterization on this motor."""
        characterizer = self.get_characterizer(config)
        return characterizer.create_characterization_command()

    def get_sysid_helper(
        self,
        mechanism_name: str = "Mechanism",
        ramp_rate_v_per_s: float = 1.0,
        step_voltage: float = 7.0,
        timeout_s: float = 10.0,
        is_bounded: bool = False,
        min_pos: float = float("-inf"),
        max_pos: float = float("inf"),
    ) -> FROGSysIdHelper:
        """Creates a FROGSysIdHelper instance wrapping WPILib SysIdRoutine and SignalLogger."""
        return FROGSysIdHelper(
            motor=self,
            mechanism_name=mechanism_name,
            ramp_rate_v_per_s=ramp_rate_v_per_s,
            step_voltage=step_voltage,
            timeout_s=timeout_s,
            is_bounded=is_bounded,
            min_pos=min_pos,
            max_pos=max_pos,
        )

    def simulation_init(
        self,
        moi: float,
        motor_model: Optional[DCMotor] = None,
        gearing: Optional[float] = None,
        measurement_std_devs: Optional[list[float]] = None,
    ):
        """Initialize physics-based simulation for this motor."""
        if motor_model is None:
            motor_model = self.motor_model or DCMotor.falcon500(1)

        if gearing is None:
            gearing = self.config.feedback.sensor_to_mechanism_ratio

        self._sim_gearing = gearing
        plant = LinearSystemId.DCMotorSystem(motor_model, moi, self._sim_gearing)

        if measurement_std_devs is None:
            measurement_std_devs = [0.0, 0.0]

        devs = np.array(measurement_std_devs) if np is not None else measurement_std_devs
        self.physim = DCMotorSim(plant, motor_model, devs)
        self.physim.setState(0.0, 0.0)

    def simulation_update(
        self,
        dt: float,
        battery_v: float,
        coupled_motors: Optional[list["FROGTalonFX"]] = None,
        max_velocity_rps: Optional[float] = None,
        velocity_sign_multiplier: int = 1,
    ):
        """Update simulation for this motor and optionally coupled motors."""
        if hasattr(self, "physim"):
            unmanaged.feed_enable(0.100)
            self.sim_state.set_supply_voltage(battery_v)
            applied_v = self.sim_state.motor_voltage
            self.physim.setInputVoltage(applied_v)
            self.physim.update(dt)

            pos_output = self.physim.getAngularPositionRotations()
            vel_output = radiansToRotations(self.physim.getAngularVelocity())

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
            if max_velocity_rps is None:
                max_velocity_rps = 100.0
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


# ------------------------------------------------------------------------------
# Gyro & Encoder Wrappers
# ------------------------------------------------------------------------------


class FROGPigeonGyro(Pigeon2):
    """Gyro class that creates an instance of the Pigeon 2.0 Gyro."""

    def __init__(self, can_id: int):
        super().__init__(can_id)
        self.reset()
        self.optimize_bus_utilization()
        self.get_yaw().set_update_frequency(100)

    def getAngleCCW(self) -> float:
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
