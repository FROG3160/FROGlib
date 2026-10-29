"""Unit tests for TalonFX characterization and gain tuning in froglib.ctre."""

import math
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

# ==============================================================================
# Headless Environment Stubs for FRC & CTRE Vendor Libraries
# ==============================================================================

try:
    import phoenix6
except ImportError:
    class GravityTypeValue:
        ELEVATOR_STATIC = 0
        ARM_COSINE = 1

    class InvertedValue:
        CLOCKWISE_POSITIVE = 0
        COUNTER_CLOCKWISE_POSITIVE = 1

    class NeutralModeValue:
        COAST = 0
        BRAKE = 1

    class SensorDirectionValue:
        COUNTER_CLOCKWISE_POSITIVE = 0
        CLOCKWISE_POSITIVE = 1

    class StaticFeedforwardSignValue:
        USE_VELOCITY_SIGN = 0
        USE_CLOSED_LOOP_SIGN = 1

    class MagnetSensorConfigs:
        def __init__(self):
            self.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
            self.absolute_sensor_discontinuity_point = 0.5
            self.magnet_offset = 0.0

        def with_sensor_direction(self, val):
            self.sensor_direction = val
            return self

        def with_absolute_sensor_discontinuity_point(self, val: float):
            self.absolute_sensor_discontinuity_point = float(val)
            return self

        def with_magnet_offset(self, val: float):
            self.magnet_offset = float(val)
            return self

        def with_absolute_sensor_range(self, val):
            return self

    class CANcoderConfiguration:
        def __init__(self):
            self.magnet_sensor = MagnetSensorConfigs()
            self.future_proof_configs = True

        def with_magnet_sensor(self, val: MagnetSensorConfigs):
            self.magnet_sensor = val
            return self

        def with_future_proof_configs(self, val: bool):
            self.future_proof_configs = bool(val)
            return self

    class CurrentLimitsConfigs:
        def __init__(self):
            self.supply_current_limit = 40.0
            self.supply_current_limit_enable = False
            self.stator_current_limit = 60.0
            self.stator_current_limit_enable = False

        def with_supply_current_limit(self, val: float):
            self.supply_current_limit = float(val)
            return self

        def with_supply_current_limit_enable(self, val: bool):
            self.supply_current_limit_enable = bool(val)
            return self

        def with_stator_current_limit(self, val: float):
            self.stator_current_limit = float(val)
            return self

        def with_stator_current_limit_enable(self, val: bool):
            self.stator_current_limit_enable = bool(val)
            return self

    class FeedbackSensorSourceValue:
        ROTOR_SENSOR = 0
        REMOTE_CANCODER = 1
        FUSED_CANCODER = 2
        SYNC_CANCODER = 3

    class FeedbackConfigs:
        def __init__(self):
            self.feedback_remote_sensor_id = 0
            self.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
            self.rotor_to_sensor_ratio = 1.0
            self.sensor_to_mechanism_ratio = 1.0

        def with_feedback_remote_sensor_id(self, val: int):
            self.feedback_remote_sensor_id = int(val)
            return self

        def with_feedback_sensor_source(self, val):
            self.feedback_sensor_source = val
            return self

        def with_rotor_to_sensor_ratio(self, val: float):
            self.rotor_to_sensor_ratio = float(val)
            return self

        def with_sensor_to_mechanism_ratio(self, val: float):
            self.sensor_to_mechanism_ratio = float(val)
            return self

    class Slot0Configs:
        def __init__(self):
            self.k_s = 0.0
            self.k_v = 0.0
            self.k_a = 0.0
            self.k_g = 0.0
            self.k_p = 0.0
            self.k_i = 0.0
            self.k_d = 0.0
            self.gravity_type = None

        def with_k_s(self, val: float):
            self.k_s = float(val)
            return self

        def with_k_v(self, val: float):
            self.k_v = float(val)
            return self

        def with_k_a(self, val: float):
            self.k_a = float(val)
            return self

        def with_k_g(self, val: float):
            self.k_g = float(val)
            return self

        def with_k_p(self, val: float):
            self.k_p = float(val)
            return self

        def with_k_i(self, val: float):
            self.k_i = float(val)
            return self

        def with_k_d(self, val: float):
            self.k_d = float(val)
            return self

        def with_gravity_type(self, val):
            self.gravity_type = val
            return self

    class Slot1Configs(Slot0Configs):
        pass

    class SoftwareLimitSwitchConfigs:
        def __init__(self):
            self.forward_soft_limit_enable = False
            self.forward_soft_limit_threshold = 0.0
            self.reverse_soft_limit_enable = False
            self.reverse_soft_limit_threshold = 0.0

        def with_forward_soft_limit_enable(self, val: bool):
            self.forward_soft_limit_enable = bool(val)
            return self

        def with_forward_soft_limit_threshold(self, val: float):
            self.forward_soft_limit_threshold = float(val)
            return self

        def with_reverse_soft_limit_enable(self, val: bool):
            self.reverse_soft_limit_enable = bool(val)
            return self

        def with_reverse_soft_limit_threshold(self, val: float):
            self.reverse_soft_limit_threshold = float(val)
            return self

    class MotorOutputConfigs:
        def __init__(self):
            self.neutral_mode = NeutralModeValue.COAST
            self.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        def with_neutral_mode(self, val):
            self.neutral_mode = val
            return self

        def with_inverted(self, val):
            self.inverted = val
            return self

    class TalonFXConfiguration:
        def __init__(self):
            self.feedback = FeedbackConfigs()
            self.slot0 = Slot0Configs()
            self.slot1 = Slot1Configs()
            self.current_limits = CurrentLimitsConfigs()
            self.motor_output = MotorOutputConfigs()
            self.software_limit_switch = SoftwareLimitSwitchConfigs()
            self.future_proof_configs = True

        def with_feedback(self, val: FeedbackConfigs):
            self.feedback = val
            return self

        def with_slot0(self, val: Slot0Configs):
            self.slot0 = val
            return self

        def with_slot1(self, val: Slot1Configs):
            self.slot1 = val
            return self

        def with_current_limits(self, val: CurrentLimitsConfigs):
            self.current_limits = val
            return self

        def with_motor_output(self, val: MotorOutputConfigs):
            self.motor_output = val
            return self

        def with_software_limit_switch(self, val: SoftwareLimitSwitchConfigs):
            self.software_limit_switch = val
            return self

        def with_future_proof_configs(self, val: bool):
            self.future_proof_configs = bool(val)
            return self

    class VoltageOut:
        def __init__(self, output: float = 0.0):
            self.output = float(output)

        def with_output(self, val: float):
            self.output = float(val)
            return self

    class PositionVoltage:
        def __init__(self, position: float = 0.0):
            self.position = float(position)
            self.slot = 0
            self.feed_forward = 0.0

        def with_position(self, val: float):
            self.position = float(val)
            return self

        def with_slot(self, val: int):
            self.slot = int(val)
            return self

        def with_feed_forward(self, val: float):
            self.feed_forward = float(val)
            return self

    class _StatusSignal:
        def __init__(self, value: float = 0.0):
            self.value = value

        def set_update_frequency(self, freq: float):
            return 0

    StatusSignal = _StatusSignal

    class SignalLogger:
        @staticmethod
        def start():
            pass

        @staticmethod
        def stop():
            pass

        @staticmethod
        def write_string(name: str, val: str):
            pass

    class _Unmanaged:
        @staticmethod
        def feed_enable(seconds: float):
            pass

    unmanaged = _Unmanaged()

    class CANBus:
        def __init__(self, name: str = "rio"):
            self.name = name

    class CANcoder:
        def __init__(self, id: int, canbus="rio"):
            self.device_id = id
            self.id = id
            self.canbus = canbus
            self.configurator = self

        def apply(self, cfg):
            self.config = cfg
            return 0

        def refresh(self, cfg):
            return 0

        def optimize_bus_utilization(self):
            pass

        def get_position(self):
            return _StatusSignal(0.0)

        def get_absolute_position(self):
            return _StatusSignal(0.0)

        def get_velocity(self):
            return _StatusSignal(0.0)

    class Pigeon2:
        def __init__(self, id: int, canbus="rio"):
            self.device_id = id
            self.id = id
            self.canbus = canbus
            self.configurator = self
            self._yaw = 0.0
            self._roll = 0.0
            self._pitch = 0.0
            self._ang_vel_z = 0.0

        def reset(self):
            self._yaw = 0.0

        def optimize_bus_utilization(self):
            pass

        def get_yaw(self):
            return _StatusSignal(self._yaw)

        def get_roll(self):
            return _StatusSignal(self._roll)

        def get_pitch(self):
            return _StatusSignal(self._pitch)

        def get_angular_velocity_z_world(self):
            return _StatusSignal(self._ang_vel_z)

        def set_yaw(self, angle: float):
            self._yaw = float(angle)

    class _SimState:
        def __init__(self):
            self.motor_voltage = 0.0
            self.supply_voltage = 12.0
            self.rotor_position = 0.0
            self.rotor_velocity = 0.0

        def set_supply_voltage(self, v: float):
            self.supply_voltage = v

        def set_raw_rotor_position(self, pos: float):
            self.rotor_position = pos

        def set_rotor_velocity(self, vel: float):
            self.rotor_velocity = vel

        def add_rotor_position(self, delta: float):
            self.rotor_position += delta

    class TalonFX:
        def __init__(self, device_id: int, canbus="rio"):
            self.device_id = device_id
            self.id = device_id
            self.canbus = canbus
            self.configurator = self
            self.sim_state = _SimState()
            self.config = TalonFXConfiguration()

        def apply(self, cfg):
            self.config = cfg
            return 0

        def refresh(self, cfg):
            return 0

        def optimize_bus_utilization(self):
            pass

        def get_position(self):
            return _StatusSignal(0.0)

        def get_velocity(self):
            return _StatusSignal(0.0)

        def get_acceleration(self):
            return _StatusSignal(0.0)

        def get_motor_voltage(self):
            return _StatusSignal(0.0)

        def get_stator_current(self):
            return _StatusSignal(0.0)

        def get_supply_voltage(self):
            return _StatusSignal(12.0)

        def get_device_temp(self):
            return _StatusSignal(25.0)

        def get_fault_field(self):
            return _StatusSignal(0.0)

        def get_rotor_velocity(self):
            return _StatusSignal(0.0)

        def set_control(self, control_req):
            if hasattr(control_req, "output"):
                self.sim_state.motor_voltage = control_req.output

    def _make_mod(name, **attrs):
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    p6 = _make_mod(
        "phoenix6",
        StatusSignal=StatusSignal,
        SignalLogger=SignalLogger,
        unmanaged=unmanaged,
    )
    p6_canbus = _make_mod("phoenix6.canbus", CANBus=CANBus)
    p6.canbus = p6_canbus

    p6_cancoder_configs = _make_mod(
        "phoenix6.configs.cancoder_configs",
        CANcoderConfiguration=CANcoderConfiguration,
        MagnetSensorConfigs=MagnetSensorConfigs,
    )
    p6_config_groups = _make_mod(
        "phoenix6.configs.config_groups",
        CurrentLimitsConfigs=CurrentLimitsConfigs,
        FeedbackConfigs=FeedbackConfigs,
        Slot0Configs=Slot0Configs,
        Slot1Configs=Slot1Configs,
        SoftwareLimitSwitchConfigs=SoftwareLimitSwitchConfigs,
    )
    p6_talon_fx_configs = _make_mod(
        "phoenix6.configs.talon_fx_configs",
        FeedbackSensorSourceValue=FeedbackSensorSourceValue,
        MotorOutputConfigs=MotorOutputConfigs,
        TalonFXConfiguration=TalonFXConfiguration,
    )
    p6_configs = _make_mod(
        "phoenix6.configs",
        cancoder_configs=p6_cancoder_configs,
        config_groups=p6_config_groups,
        talon_fx_configs=p6_talon_fx_configs,
    )
    p6.configs = p6_configs

    p6_controls = _make_mod(
        "phoenix6.controls",
        PositionVoltage=PositionVoltage,
        VoltageOut=VoltageOut,
    )
    p6.controls = p6_controls

    p6_cancoder = _make_mod("phoenix6.hardware.cancoder", CANcoder=CANcoder)
    p6_pigeon2 = _make_mod("phoenix6.hardware.pigeon2", Pigeon2=Pigeon2)
    p6_talon_fx = _make_mod("phoenix6.hardware.talon_fx", TalonFX=TalonFX)
    p6_hw = _make_mod(
        "phoenix6.hardware",
        cancoder=p6_cancoder,
        pigeon2=p6_pigeon2,
        talon_fx=p6_talon_fx,
    )
    p6.hardware = p6_hw

    p6_spn_enums = _make_mod(
        "phoenix6.signals.spn_enums",
        GravityTypeValue=GravityTypeValue,
        InvertedValue=InvertedValue,
        NeutralModeValue=NeutralModeValue,
        SensorDirectionValue=SensorDirectionValue,
        StaticFeedforwardSignValue=StaticFeedforwardSignValue,
    )
    p6_signals = _make_mod("phoenix6.signals", spn_enums=p6_spn_enums)
    p6.signals = p6_signals

try:
    import ntcore
except ImportError:
    class _DummyPub:
        def set(self, val):
            pass

        def setDefault(self, val):
            pass

    class _DummyTop:
        def publish(self):
            return _DummyPub()

    class _DummyNT:
        def getStringTopic(self, _):
            return _DummyTop()

        def getFloatTopic(self, _):
            return _DummyTop()

    class NetworkTableInstance:
        @staticmethod
        def getDefault():
            return _DummyNT()

    nt_mod = types.ModuleType("ntcore")
    nt_mod.NetworkTableInstance = NetworkTableInstance
    sys.modules["ntcore"] = nt_mod

try:
    import wpimath
except ImportError:
    class Rotation2d:
        def __init__(self, value: float = 0.0):
            self.value = float(value)

        @staticmethod
        def fromDegrees(deg: float):
            return Rotation2d(math.radians(deg))

        @staticmethod
        def fromRadians(rad: float):
            return Rotation2d(rad)

        def degrees(self):
            return math.degrees(self.value)

        def radians(self):
            return self.value

    class DCMotor:
        @staticmethod
        def falcon500(count: int = 1):
            return DCMotor()

        @staticmethod
        def krakenX60(count: int = 1):
            return DCMotor()

    class LinearSystemId:
        @staticmethod
        def DCMotorSystem(motor, moi, gearing):
            return object()

    def radiansToRotations(rad: float) -> float:
        return rad / (2.0 * math.pi)

    def volts(v: float) -> float:
        return float(v)

    def seconds(s: float) -> float:
        return float(s)

    def volts_per_second(v: float) -> float:
        return float(v)

    class SysIdRoutineLog:
        class State:
            kQuasistaticForward = "QuasistaticForward"
            kQuasistaticReverse = "QuasistaticReverse"
            kDynamicForward = "DynamicForward"
            kDynamicReverse = "DynamicReverse"
            kNone = "None"

    wpimath_mod = types.ModuleType("wpimath")
    geom_mod = types.ModuleType("wpimath.geometry")
    geom_mod.Rotation2d = Rotation2d
    for _geom_cls in (
        "Transform3d",
        "Translation3d",
        "Rotation3d",
        "Pose3d",
        "Pose2d",
        "Transform2d",
        "Translation2d",
    ):
        setattr(geom_mod, _geom_cls, type(_geom_cls, (), {}))
    sys_mod = types.ModuleType("wpimath.system")
    plant_mod = types.ModuleType("wpimath.system.plant")
    plant_mod.DCMotor = DCMotor
    plant_mod.LinearSystemId = LinearSystemId
    sys_mod.plant = plant_mod
    units_mod = types.ModuleType("wpimath.units")
    units_mod.radiansToRotations = radiansToRotations
    units_mod.volts = volts
    units_mod.seconds = seconds
    units_mod.volts_per_second = volts_per_second
    sysid_mod = types.ModuleType("wpimath.sysid")
    sysid_mod.SysIdRoutineLog = SysIdRoutineLog

    wpimath_mod.geometry = geom_mod
    wpimath_mod.system = sys_mod
    wpimath_mod.units = units_mod
    wpimath_mod.sysid = sysid_mod

    sys.modules["wpimath"] = wpimath_mod
    sys.modules["wpimath.geometry"] = geom_mod
    sys.modules["wpimath.system"] = sys_mod
    sys.modules["wpimath.system.plant"] = plant_mod
    sys.modules["wpimath.units"] = units_mod
    sys.modules["wpimath.sysid"] = sysid_mod

try:
    import wpilib
except ImportError:
    class DCMotorSim:
        def __init__(self, plant=None, motor=None, measurement_std_devs=None):
            self._pos = 0.0
            self._vel = 0.0
            self._voltage = 0.0

        def setState(self, pos: float, vel: float):
            self._pos = float(pos)
            self._vel = float(vel)

        def setInputVoltage(self, v: float):
            self._voltage = float(v)

        def update(self, dt: float):
            pass

        def getAngularPositionRotations(self) -> float:
            return self._pos

        def getAngularVelocity(self) -> float:
            return self._vel

    wpilib_mod = types.ModuleType("wpilib")
    sim_mod = types.ModuleType("wpilib.simulation")
    sim_mod.DCMotorSim = DCMotorSim
    wpilib_mod.simulation = sim_mod
    sys.modules["wpilib"] = wpilib_mod
    sys.modules["wpilib.simulation"] = sim_mod

try:
    import commands2
except ImportError:
    class Command:
        pass

    class FunctionalCommand(Command):
        def __init__(self, onInit=None, onExecute=None, onEnd=None, isFinished=None, *requirements):
            self._on_init = onInit
            self._on_execute = onExecute
            self._on_end = onEnd
            self._is_finished = isFinished

        def initialize(self):
            if self._on_init:
                self._on_init()

        def execute(self):
            if self._on_execute:
                self._on_execute()

        def end(self, interrupted: bool):
            if self._on_end:
                self._on_end(interrupted)

        def isFinished(self) -> bool:
            if self._is_finished:
                return bool(self._is_finished())
            return False

    class SysIdRoutine:
        class Direction:
            kForward = 1
            kReverse = -1

        class Config:
            def __init__(self, rampRate=None, stepVoltage=None, timeout=None, recordState=None):
                self.rampRate = rampRate
                self.stepVoltage = stepVoltage
                self.timeout = timeout
                self.recordState = recordState

        class Mechanism:
            def __init__(self, drive=None, log=None, subsystem=None, name=""):
                self.drive = drive
                self.log = log

        def __init__(self, config=None, mechanism=None):
            self.config = config
            self.mechanism = mechanism

        def quasistatic(self, direction):
            cmd = MagicMock(spec=Command)
            cmd.execute = MagicMock()
            cmd.isFinished = MagicMock(return_value=False)
            return cmd

        def dynamic(self, direction):
            cmd = MagicMock(spec=Command)
            cmd.execute = MagicMock()
            cmd.isFinished = MagicMock(return_value=False)
            return cmd

    cmd2_mod = types.ModuleType("commands2")
    cmd2_mod.Command = Command
    cmd2_mod.FunctionalCommand = FunctionalCommand
    sysid_mod = types.ModuleType("commands2.sysid")
    sysid_mod.SysIdRoutine = SysIdRoutine
    cmd2_mod.sysid = sysid_mod

    sys.modules["commands2"] = cmd2_mod
    sys.modules["commands2.sysid"] = sysid_mod

try:
    import numpy
except ImportError:
    class _NumpyStub:
        @staticmethod
        def array(x, *args, **kwargs):
            return list(x) if hasattr(x, "__iter__") else x

        ndarray = list

    np_mod = types.ModuleType("numpy")
    np_mod.array = _NumpyStub.array
    np_mod.ndarray = _NumpyStub.ndarray
    sys.modules["numpy"] = np_mod

# ==============================================================================
# FROGlib CTRE Imports
# ==============================================================================

from froglib.ctre import (
    MAX_FALCON_RPM,
    MAX_KRAKEN_X60_RPM,
    CharacterizationConfig,
    CurrentLimitsConfigs,
    FeedbackSensorSourceValue,
    FROGCANCoderConfig,
    FROGFeedbackConfig,
    FROGSysIdHelper,
    FROGTalonFX,
    FROGTalonFXConfig,
    SensorDirectionValue,
    Slot0Configs,
    Slot1Configs,
    TalonFXCharacterizer,
    TalonFXConfiguration,
    TuningResult,
    calculate_feedback_gains,
    get_frog_cancoder_config,
    get_frog_talon_config,
    linear_regression,
)


class TestTuningMath(unittest.TestCase):
    """Tests for mathematical algorithms used in gain tuning."""

    def test_linear_regression_perfect_fit(self):
        """Tests linear regression with a known ideal line: y = 0.12 * x + 0.25."""
        x_data = [10.0, 20.0, 40.0, 60.0, 80.0]
        true_kv = 0.12
        true_offset = 0.25
        y_data = [true_kv * x + true_offset for x in x_data]

        slope, intercept, r_squared = linear_regression(x_data, y_data)

        self.assertAlmostEqual(slope, true_kv, places=4)
        self.assertAlmostEqual(intercept, true_offset, places=4)
        self.assertAlmostEqual(r_squared, 1.0, places=4)

    def test_linear_regression_noisy_data(self):
        """Tests linear regression with slight noise, verifying R² < 1.0 but close."""
        x_data = [10.0, 20.0, 30.0, 40.0, 50.0]
        y_data = [1.2, 2.35, 3.65, 4.85, 6.0]

        slope, intercept, r_squared = linear_regression(x_data, y_data)

        self.assertGreater(slope, 0.11)
        self.assertLess(slope, 0.13)
        self.assertGreater(r_squared, 0.99)

    def test_linear_regression_edge_cases(self):
        """Tests linear regression handles empty or single-item lists gracefully."""
        slope, intercept, r_sq = linear_regression([], [])
        self.assertEqual(slope, 0.0)
        self.assertEqual(intercept, 0.0)

        slope, intercept, r_sq = linear_regression([5.0], [1.0])
        self.assertEqual(slope, 0.0)

    def test_calculate_feedback_gains(self):
        """Tests 2nd-order pole placement feedback synthesis for kP and kD."""
        k_v = 0.12
        k_a = 0.008
        omega_n = 25.0
        zeta = 1.0

        k_p, k_d = calculate_feedback_gains(
            k_v=k_v,
            k_a=k_a,
            target_bandwidth_rad_s=omega_n,
            damping_ratio=zeta,
        )

        expected_kp = 0.008 * (25.0**2)
        self.assertAlmostEqual(k_p, expected_kp, places=4)

        expected_kd = (2.0 * zeta * omega_n * k_a) - k_v
        self.assertAlmostEqual(k_d, expected_kd, places=4)

    def test_calculate_feedback_gains_negative_kd_clamped(self):
        """Verifies kD is clamped to 0.0 when natural back-EMF damping exceeds target damping."""
        k_v = 0.50
        k_a = 0.002
        omega_n = 10.0
        zeta = 1.0

        k_p, k_d = calculate_feedback_gains(k_v, k_a, target_bandwidth_rad_s=omega_n, damping_ratio=zeta)

        self.assertGreater(k_p, 0.0)
        self.assertEqual(k_d, 0.0)


class TestTuningResult(unittest.TestCase):
    """Tests for TuningResult data structure and export methods."""

    def setUp(self):
        self.result = TuningResult(
            mechanism_name="ShooterFlywheel",
            k_s=0.235,
            k_v=0.118,
            k_a=0.0075,
            k_g=None,
            k_p=4.68,
            k_d=0.182,
            r_squared=0.998,
            gearing=1.0,
            is_mechanism_units=True,
        )

    def test_to_slot0_configs(self):
        """Tests export to Phoenix 6 Slot0Configs."""
        slot0 = self.result.to_slot0_configs()
        self.assertAlmostEqual(slot0.k_s, 0.235)
        self.assertAlmostEqual(slot0.k_v, 0.118)
        self.assertAlmostEqual(slot0.k_a, 0.0075)
        self.assertAlmostEqual(slot0.k_p, 4.68)
        self.assertAlmostEqual(slot0.k_d, 0.182)

    def test_format_report(self):
        """Tests report string formatting."""
        report = self.result.format_report()
        self.assertIn("ShooterFlywheel", report)
        self.assertIn("0.2350 V", report)
        self.assertIn("0.9980", report)

    def test_to_python_code(self):
        """Tests code generation output."""
        code = self.result.to_python_code()
        self.assertIn("cfg.slot0.k_s = 0.2350", code)
        self.assertIn("cfg.slot0.k_v = 0.1180", code)


class TestGearingAndSafety(unittest.TestCase):
    """Tests for gear ratio validation and safety limits."""

    def test_validate_gearing_correct(self):
        """Verifies validate_gearing approves when measured kV aligns with gearing."""
        mock_motor = MagicMock()
        mock_motor.config.feedback.sensor_to_mechanism_ratio = 10.0

        characterizer = TalonFXCharacterizer(mock_motor)
        is_valid, msg = characterizer.validate_gearing(measured_kv=1.22)
        self.assertTrue(is_valid)

    def test_validate_gearing_mismatch(self):
        """Verifies validate_gearing warns when measured kV does not align with gearing."""
        mock_motor = MagicMock()
        mock_motor.config.feedback.sensor_to_mechanism_ratio = 10.0

        characterizer = TalonFXCharacterizer(mock_motor)
        is_valid, msg = characterizer.validate_gearing(measured_kv=0.12)
        self.assertFalse(is_valid)
        self.assertIn("WARNING", msg)

    def test_safety_bounds_trigger(self):
        """Tests that bounded mechanism stops before exceeding limits."""
        mock_motor = MagicMock()
        mock_motor.get_position.return_value.value = 19.5
        mock_motor.get_velocity.return_value.value = 1.0
        mock_motor.get_stator_current.return_value.value = 10.0

        config = CharacterizationConfig(
            is_bounded=True,
            min_test_pos=0.0,
            max_test_pos=20.0,
            soft_limit_buffer=1.0,
        )

        characterizer = TalonFXCharacterizer(mock_motor, config)
        characterizer._current_voltage = 5.0

        is_safe = characterizer.check_safety_guards()
        self.assertFalse(is_safe)
        self.assertEqual(characterizer.state, TalonFXCharacterizer.State.ABORTED)


class TestFROGTalonFXConvenienceMethods(unittest.TestCase):
    """Tests convenience methods added to FROGTalonFX for tuning integration."""

    def test_talonfx_helpers(self):
        """Verifies FROGTalonFX creates characterizer, command, and SysId helpers."""
        motor = FROGTalonFX(id=1)
        config = CharacterizationConfig(mechanism_name="TestMotor")

        characterizer = motor.get_characterizer(config)
        self.assertIsInstance(characterizer, TalonFXCharacterizer)
        self.assertEqual(characterizer.config.mechanism_name, "TestMotor")

        sysid_helper = motor.get_sysid_helper(mechanism_name="TestSysId")
        self.assertIsInstance(sysid_helper, FROGSysIdHelper)
        self.assertEqual(sysid_helper.mechanism_name, "TestSysId")


class _SignalValue:
    def __init__(self, value: float = 0.0):
        self.value = value


class SimulatedTalonFXPlant:
    """Physics-based simulated TalonFX + mechanism plant for end-to-end testing.

    Models the continuous-time motor/mechanism differential equation:
        V = k_s * sign(omega) + k_v * omega + k_a * alpha
    """

    def __init__(
        self,
        true_ks: float = 0.24,
        true_kv: float = 0.12,
        true_ka: float = 0.015,
        gearing: float = 1.0,
        initial_position: float = 0.0,
        winding_resistance_ohms: float = 0.08,
        expose_acceleration_signal: bool = True,
    ):
        self.true_ks = true_ks
        self.true_kv = true_kv
        self.true_ka = true_ka
        self.winding_resistance_ohms = winding_resistance_ohms
        self.expose_acceleration_signal = expose_acceleration_signal

        self.config = TalonFXConfiguration()
        self.config.feedback.sensor_to_mechanism_ratio = gearing

        self.position = initial_position
        self.velocity = 0.0
        self.acceleration = 0.0
        self.applied_voltage = 0.0
        self.stator_current = 0.0
        self.jammed = False

        self.active_profile = FROGTalonFX.SignalProfile.BASIC
        self.applied_configs = []

        self.configurator = self

    def apply(self, cfg):
        self.applied_configs.append(cfg)
        if isinstance(cfg, Slot1Configs):
            self.config.slot1 = cfg
        elif isinstance(cfg, Slot0Configs):
            self.config.slot0 = cfg
        elif isinstance(cfg, CurrentLimitsConfigs):
            self.config.current_limits = cfg

    def apply_usage_mode(self, profile: FROGTalonFX.SignalProfile):
        self.active_profile = profile

    def set_control(self, control_req):
        self.applied_voltage = getattr(control_req, "output", 0.0)

    def get_position(self):
        return _SignalValue(self.position)

    def get_velocity(self):
        return _SignalValue(self.velocity)

    def get_acceleration(self):
        return _SignalValue(self.acceleration if self.expose_acceleration_signal else 0.0)

    def get_stator_current(self):
        return _SignalValue(self.stator_current)

    def step_physics(self, dt: float):
        """Advances the simulated physics plant by dt seconds using sub-stepping."""
        substeps = max(1, int(math.ceil(dt / 0.0005)))
        sub_dt = dt / substeps

        for _ in range(substeps):
            if self.jammed:
                self.velocity = 0.0
                self.acceleration = 0.0
                raw_current = abs(self.applied_voltage) / self.winding_resistance_ohms
            else:
                v_app = self.applied_voltage
                if abs(self.velocity) < 1e-4:
                    # Static stiction regime
                    if abs(v_app) <= self.true_ks:
                        self.velocity = 0.0
                        self.acceleration = 0.0
                    else:
                        net_v = v_app - math.copysign(self.true_ks, v_app)
                        self.acceleration = net_v / self.true_ka
                        self.velocity += self.acceleration * sub_dt
                else:
                    # Dynamic motion regime
                    friction_v = math.copysign(self.true_ks, self.velocity)
                    back_emf_v = self.true_kv * self.velocity
                    net_v = v_app - friction_v - back_emf_v
                    self.acceleration = net_v / self.true_ka
                    next_vel = self.velocity + self.acceleration * sub_dt
                    # Zero-crossing deadband when coasting to a stop with 0V applied
                    if abs(v_app) <= self.true_ks and (next_vel * self.velocity <= 0.0):
                        self.velocity = 0.0
                        self.acceleration = 0.0
                    else:
                        self.velocity = next_vel

                self.position += self.velocity * sub_dt
                if abs(self.velocity) >= 1e-4:
                    friction_v = math.copysign(self.true_ks, self.velocity)
                    back_emf_v = self.true_kv * self.velocity
                    self.acceleration = (v_app - friction_v - back_emf_v) / self.true_ka
                back_emf = self.true_kv * self.velocity
                raw_current = abs(v_app - back_emf) / self.winding_resistance_ohms

            if (
                self.config.current_limits.stator_current_limit_enable
                and self.config.current_limits.stator_current_limit > 0.0
            ):
                self.stator_current = min(
                    raw_current, self.config.current_limits.stator_current_limit
                )
            else:
                self.stator_current = raw_current


class TestEndToEndPhysicsSimulation(unittest.TestCase):
    """End-to-end state-machine tests using a simulated TalonFX physics plant."""

    def _run_characterizer_loop(
        self,
        characterizer: TalonFXCharacterizer,
        plant: SimulatedTalonFXPlant,
        dt: float = 0.004,
        max_sim_time: float = 30.0,
    ):
        """Runs the characterizer state machine against the physics plant with a virtual clock."""
        sim_time = 100.0
        visited_states = []

        with patch("froglib.ctre.time.monotonic", side_effect=lambda: sim_time):
            characterizer.setup()
            visited_states.append(characterizer.state)

            steps = int(max_sim_time / dt)
            for _ in range(steps):
                sim_time += dt
                plant.step_physics(dt)
                finished = characterizer.periodic()
                if not visited_states or visited_states[-1] != characterizer.state:
                    visited_states.append(characterizer.state)
                if finished:
                    break

        return visited_states

    def test_continuous_flywheel_characterization_end_to_end(self):
        """Verifies full state-machine execution and accurate gain recovery on a simulated flywheel."""
        plant = SimulatedTalonFXPlant(
            true_ks=0.24,
            true_kv=0.12,
            true_ka=0.015,
            gearing=1.0,
        )
        config = CharacterizationConfig(
            mechanism_name="SimFlywheel",
            is_bounded=False,
            ramp_rate_v_per_s=0.5,
            kv_test_voltages=[2.0, 4.0, 6.0, 8.0, 10.0],
            kv_settle_time_s=0.45,
            ka_step_voltage=6.0,
            ka_step_duration_s=0.12,
            target_bandwidth_rad_s=25.0,
            damping_ratio=1.0,
            auto_apply=True,
            target_slot=0,
        )
        characterizer = TalonFXCharacterizer(plant, config)

        visited_states = self._run_characterizer_loop(characterizer, plant)

        expected_state_order = [
            TalonFXCharacterizer.State.KS_FORWARD,
            TalonFXCharacterizer.State.RETURN_HOME_1,
            TalonFXCharacterizer.State.KS_REVERSE,
            TalonFXCharacterizer.State.RETURN_HOME_2,
            TalonFXCharacterizer.State.KV_SWEEP,
            TalonFXCharacterizer.State.RETURN_HOME_3,
            TalonFXCharacterizer.State.KA_STEP,
            TalonFXCharacterizer.State.RETURN_HOME_4,
            TalonFXCharacterizer.State.COMPUTE_AND_FINISH,
        ]
        self.assertEqual(visited_states, expected_state_order)

        result = characterizer.get_result()
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.k_s, plant.true_ks, delta=0.025)
        self.assertAlmostEqual(result.k_v, plant.true_kv, delta=0.005)
        self.assertAlmostEqual(result.k_a, plant.true_ka, delta=0.002)
        self.assertGreater(result.r_squared, 0.999)

        # Verify Slot 0 hot-apply and teardown restoration
        self.assertAlmostEqual(plant.config.slot0.k_s, result.k_s, places=4)
        self.assertAlmostEqual(plant.config.slot0.k_v, result.k_v, places=4)
        self.assertAlmostEqual(plant.config.slot0.k_a, result.k_a, places=4)
        self.assertAlmostEqual(plant.config.slot0.k_p, result.k_p, places=4)
        self.assertAlmostEqual(plant.config.slot0.k_d, result.k_d, places=4)
        self.assertEqual(plant.applied_voltage, 0.0)
        self.assertEqual(plant.active_profile, FROGTalonFX.SignalProfile.BASIC)
        self.assertEqual(plant.config.current_limits.stator_current_limit, 40.0)

    def test_bounded_mechanism_characterization_and_return_home(self):
        """Verifies a bounded elevator mechanism stays within travel limits and returns home between stages."""
        plant = SimulatedTalonFXPlant(
            true_ks=0.30,
            true_kv=0.60,
            true_ka=0.025,
            gearing=5.0,
            initial_position=5.0,
        )
        config = CharacterizationConfig(
            mechanism_name="SimElevator",
            is_bounded=True,
            min_test_pos=0.0,
            max_test_pos=25.0,
            safe_home_pos=5.0,
            soft_limit_buffer=0.5,
            step_timeout_s=8.0,
            ramp_rate_v_per_s=1.0,
            kv_test_voltages=[2.0, 4.0, 6.0],
            kv_settle_time_s=0.30,
            ka_step_voltage=5.0,
            ka_step_duration_s=0.10,
            auto_apply=True,
            target_slot=1,
        )
        characterizer = TalonFXCharacterizer(plant, config)

        visited_states = self._run_characterizer_loop(
            characterizer, plant, max_sim_time=45.0
        )

        self.assertEqual(
            visited_states[-1], TalonFXCharacterizer.State.COMPUTE_AND_FINISH
        )
        # Mechanism should have returned within 0.1 rotations of safe_home_pos (5.0) at completion
        self.assertAlmostEqual(plant.position, config.safe_home_pos, delta=0.1)

        result = characterizer.get_result()
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.k_s, plant.true_ks, delta=0.03)
        self.assertAlmostEqual(result.k_v, plant.true_kv, delta=0.02)
        self.assertAlmostEqual(result.k_a, plant.true_ka, delta=0.005)
        self.assertEqual(result.gearing, 5.0)

        # Verify target_slot=1 applied gains to slot1
        self.assertAlmostEqual(plant.config.slot1.k_v, result.k_v, places=4)
        self.assertAlmostEqual(plant.config.slot1.k_p, result.k_p, places=4)

    def test_ka_fallback_without_acceleration_signal(self):
        """Verifies kA estimation still works when motor.get_acceleration() returns 0.0."""
        plant = SimulatedTalonFXPlant(
            true_ks=0.20,
            true_kv=0.12,
            true_ka=0.020,
            expose_acceleration_signal=False,
        )
        config = CharacterizationConfig(
            mechanism_name="FallbackAccelMotor",
            kv_test_voltages=[2.0, 4.0, 6.0],
            kv_settle_time_s=0.45,
            ka_step_voltage=6.0,
            ka_step_duration_s=0.05,
        )
        characterizer = TalonFXCharacterizer(plant, config)

        self._run_characterizer_loop(characterizer, plant)
        result = characterizer.get_result()
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.k_a, plant.true_ka, delta=0.004)


class TestSafetyAndFaultInjection(unittest.TestCase):
    """Tests for stall detection, reverse limits, timeouts, and SysId bounded protection."""

    def test_reverse_safety_bounds_trigger(self):
        """Verifies reverse travel limit triggers emergency abort when moving negatively."""
        mock_motor = MagicMock()
        mock_motor.get_position.return_value.value = 0.4
        mock_motor.get_velocity.return_value.value = -1.0
        mock_motor.get_stator_current.return_value.value = 10.0

        config = CharacterizationConfig(
            is_bounded=True,
            min_test_pos=0.0,
            max_test_pos=20.0,
            soft_limit_buffer=0.5,
        )
        characterizer = TalonFXCharacterizer(mock_motor, config)
        characterizer._current_voltage = -3.0

        is_safe = characterizer.check_safety_guards()
        self.assertFalse(is_safe)
        self.assertEqual(characterizer.state, TalonFXCharacterizer.State.ABORTED)
        self.assertIn("Reverse travel limit reached", characterizer._abort_reason)

    def test_stall_detection_triggers_emergency_abort(self):
        """Verifies that a jammed motor exceeding stall current for stall_duration_threshold_s aborts."""
        plant = SimulatedTalonFXPlant()
        plant.jammed = True

        config = CharacterizationConfig(
            mechanism_name="JammedMechanism",
            stator_current_limit_amps=30.0,
            stall_detection_enabled=True,
            stall_current_threshold=18.0,
            stall_velocity_threshold=0.05,
            stall_duration_threshold_s=0.15,
            ramp_rate_v_per_s=2.0,
            step_timeout_s=5.0,
        )
        characterizer = TalonFXCharacterizer(plant, config)

        sim_time = 50.0
        dt = 0.01
        with patch("froglib.ctre.time.monotonic", side_effect=lambda: sim_time):
            characterizer.setup()
            for _ in range(200):
                sim_time += dt
                plant.step_physics(dt)
                if characterizer.periodic():
                    break

        self.assertEqual(characterizer.state, TalonFXCharacterizer.State.ABORTED)
        self.assertIn("Motor stall detected", characterizer._abort_reason)
        self.assertEqual(plant.applied_voltage, 0.0)
        self.assertEqual(plant.active_profile, FROGTalonFX.SignalProfile.BASIC)

    def test_step_timeout_triggers_emergency_abort(self):
        """Verifies that exceeding step_timeout_s aborts the routine."""
        plant = SimulatedTalonFXPlant(true_ks=15.0)  # Unreachable breakaway voltage
        config = CharacterizationConfig(
            mechanism_name="TimeoutMechanism",
            max_voltage=10.0,
            stall_detection_enabled=False,
            step_timeout_s=1.0,
            ramp_rate_v_per_s=0.5,
        )
        characterizer = TalonFXCharacterizer(plant, config)

        sim_time = 10.0
        dt = 0.05
        with patch("froglib.ctre.time.monotonic", side_effect=lambda: sim_time):
            characterizer.setup()
            for _ in range(50):
                sim_time += dt
                plant.step_physics(dt)
                if characterizer.periodic():
                    break

        self.assertEqual(characterizer.state, TalonFXCharacterizer.State.ABORTED)
        self.assertIn("exceeded maximum timeout", characterizer._abort_reason)
        self.assertEqual(plant.applied_voltage, 0.0)

    def test_sysid_helper_bounded_drive_and_lifecycle(self):
        """Verifies FROGSysIdHelper clamps voltage at position bounds and manages signal profiles."""
        plant = SimulatedTalonFXPlant(initial_position=10.0)
        helper = FROGSysIdHelper(
            motor=plant,
            mechanism_name="BoundedArm",
            is_bounded=True,
            min_pos=2.0,
            max_pos=15.0,
        )

        # Within bounds: voltage passes through
        helper._drive_voltage(4.0)
        self.assertEqual(plant.applied_voltage, 4.0)

        # At upper bound: positive voltage is clamped to 0.0, negative voltage is allowed
        plant.position = 15.0
        helper._drive_voltage(4.0)
        self.assertEqual(plant.applied_voltage, 0.0)
        helper._drive_voltage(-4.0)
        self.assertEqual(plant.applied_voltage, -4.0)

        # At lower bound: negative voltage is clamped to 0.0, positive voltage is allowed
        plant.position = 2.0
        helper._drive_voltage(-4.0)
        self.assertEqual(plant.applied_voltage, 0.0)
        helper._drive_voltage(4.0)
        self.assertEqual(plant.applied_voltage, 4.0)

        # Lifecycle hooks switch between TUNING and BASIC profiles and zero output
        helper._on_start()
        self.assertEqual(plant.active_profile, FROGTalonFX.SignalProfile.TUNING)
        helper._on_end(interrupted=True, cmd=None)
        self.assertEqual(plant.active_profile, FROGTalonFX.SignalProfile.BASIC)
        self.assertEqual(plant.applied_voltage, 0.0)


class TestCTREConfigurations(unittest.TestCase):
    """Tests for FROGlib CTRE configurations and constants."""

    def test_get_frog_talon_config_current_limits(self):
        """Verifies get_frog_talon_config sets 40A supply and 60A stator limits."""
        cfg = get_frog_talon_config()
        self.assertIsInstance(cfg, TalonFXConfiguration)
        self.assertTrue(cfg.current_limits.supply_current_limit_enable)
        self.assertEqual(cfg.current_limits.supply_current_limit, 40)
        self.assertTrue(cfg.current_limits.stator_current_limit_enable)
        self.assertEqual(cfg.current_limits.stator_current_limit, 60)

    def test_cancoder_configs(self):
        """Verifies get_frog_cancoder_config and FROGCANCoderConfig set offset and discontinuity point."""
        cfg1 = get_frog_cancoder_config(steer_offset=-0.25)
        self.assertIsInstance(cfg1, CANcoderConfiguration)
        self.assertEqual(cfg1.magnet_sensor.magnet_offset, -0.25)
        self.assertEqual(cfg1.magnet_sensor.absolute_sensor_discontinuity_point, 0.5)
        self.assertEqual(
            cfg1.magnet_sensor.sensor_direction,
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
        )

        cfg2 = FROGCANCoderConfig(steer_offset=-0.25)
        self.assertIsInstance(cfg2, CANcoderConfiguration)
        self.assertEqual(cfg2.magnet_sensor.magnet_offset, -0.25)
        self.assertEqual(cfg2.magnet_sensor.absolute_sensor_discontinuity_point, 0.5)
        self.assertEqual(
            cfg2.magnet_sensor.sensor_direction,
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
        )

    def test_frog_feedback_and_talon_fx_config(self):
        """Verifies FROGFeedbackConfig and FROGTalonFXConfig initialize feedback and slot configs properly."""
        feedback = FROGFeedbackConfig(
            remote_sensor_id=14,
            sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
            rotor_to_sensor_ratio=2.5,
            sensor_to_mechanism_ratio=10.0,
        )
        self.assertEqual(feedback.feedback_remote_sensor_id, 14)
        self.assertEqual(
            feedback.feedback_sensor_source, FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        self.assertEqual(feedback.rotor_to_sensor_ratio, 2.5)
        self.assertEqual(feedback.sensor_to_mechanism_ratio, 10.0)

        slot0 = Slot0Configs().with_k_p(5.0)
        slot1 = Slot1Configs().with_k_v(1.2)
        talon_cfg = FROGTalonFXConfig(
            feedback_config=feedback,
            slot0gains=slot0,
            slot1gains=slot1,
        )
        self.assertIsInstance(talon_cfg, TalonFXConfiguration)
        self.assertEqual(talon_cfg.feedback.feedback_remote_sensor_id, 14)
        self.assertEqual(talon_cfg.feedback.rotor_to_sensor_ratio, 2.5)
        self.assertEqual(talon_cfg.feedback.sensor_to_mechanism_ratio, 10.0)
        self.assertEqual(talon_cfg.slot0.k_p, 5.0)
        self.assertEqual(talon_cfg.slot1.k_v, 1.2)
        # Verify safe current limits are also applied
        self.assertTrue(talon_cfg.current_limits.supply_current_limit_enable)
        self.assertEqual(talon_cfg.current_limits.supply_current_limit, 40)
        self.assertTrue(talon_cfg.current_limits.stator_current_limit_enable)
        self.assertEqual(talon_cfg.current_limits.stator_current_limit, 60)

    def test_frog_configs_default_instantiation(self):
        """Verifies default instantiation of FROGFeedbackConfig and FROGTalonFXConfig."""
        feedback = FROGFeedbackConfig()
        self.assertEqual(feedback.feedback_remote_sensor_id, 0)
        self.assertEqual(
            feedback.feedback_sensor_source, FeedbackSensorSourceValue.ROTOR_SENSOR
        )
        self.assertEqual(feedback.rotor_to_sensor_ratio, 1.0)
        self.assertEqual(feedback.sensor_to_mechanism_ratio, 1.0)

        talon_cfg = FROGTalonFXConfig()
        self.assertIsInstance(talon_cfg.feedback, FROGFeedbackConfig)
        self.assertIsInstance(talon_cfg.slot0, Slot0Configs)
        self.assertIsInstance(talon_cfg.slot1, Slot1Configs)
        self.assertTrue(talon_cfg.current_limits.supply_current_limit_enable)
        self.assertEqual(talon_cfg.current_limits.supply_current_limit, 40)
        self.assertTrue(talon_cfg.current_limits.stator_current_limit_enable)
        self.assertEqual(talon_cfg.current_limits.stator_current_limit, 60)

    def test_rpm_constants(self):
        """Verifies Falcon and Kraken RPM constants."""
        self.assertEqual(MAX_FALCON_RPM, 6380)
        self.assertEqual(MAX_KRAKEN_X60_RPM, 6000)


class TestRefactoredModules(unittest.TestCase):
    """Tests for deduplicated helpers across navx, subsystem, and utils."""

    def test_constrain_radians_and_navx_angle_constrained(self):
        """Verifies constrain_radians wraps angles to [-pi, pi] and FROGNavXGyro.getAngleConstrained wraps to [-180, 180]."""
        from froglib.utils import constrain_radians

        self.assertAlmostEqual(constrain_radians(3.0 * math.pi), math.pi, places=4)
        self.assertAlmostEqual(constrain_radians(-2.5 * math.pi), -0.5 * math.pi, places=4)

        # Test FROGNavXGyro.getAngleConstrained with mocked getAngleCCW
        import types
        import sys
        if "navx" not in sys.modules:
            navx_mod = types.ModuleType("navx")
            navx_mod.AHRS = MagicMock()
            sys.modules["navx"] = navx_mod
        from froglib.navx import FROGNavXGyro, FROGGyro

        self.assertIs(FROGGyro, FROGNavXGyro)
        gyro = FROGNavXGyro()
        gyro.getAngleCCW = MagicMock(return_value=270.0)
        self.assertAlmostEqual(gyro.getAngleConstrained(), -90.0, places=4)


if __name__ == "__main__":
    unittest.main()
