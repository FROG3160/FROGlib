"""Unit tests for TalonFX characterization and gain tuning in froglib.ctre."""

import math
import unittest
from unittest.mock import MagicMock

from froglib.ctre import (
    CharacterizationConfig,
    FROGSysIdHelper,
    FROGTalonFX,
    TalonFXCharacterizer,
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


if __name__ == "__main__":
    unittest.main()
