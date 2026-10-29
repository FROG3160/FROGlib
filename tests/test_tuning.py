"""Unit tests for TalonFX characterization and gain tuning in froglib.ctre."""

import math
import unittest
from unittest.mock import MagicMock, patch

from froglib.ctre import (
    CharacterizationConfig,
    CurrentLimitsConfigs,
    FROGSysIdHelper,
    FROGTalonFX,
    Slot0Configs,
    Slot1Configs,
    TalonFXCharacterizer,
    TalonFXConfiguration,
    TuningResult,
    calculate_feedback_gains,
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


if __name__ == "__main__":
    unittest.main()
