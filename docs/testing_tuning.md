# Testing & Verifying Automated Motor Tuning

This guide provides a comprehensive walkthrough for testing, simulating, and verifying FROGlib's automated motor characterization and gain tuning subsystem (`froglib.ctre`).

Testing is structured in three progressive tiers:
1. **Repository Unit & Physics Simulation Suite**: Fast, deterministic test execution using `SimulatedTalonFXPlant` to validate algorithms, state transitions, and safety fault-injections without hardware.
2. **Robot Project Simulation (WPILib Sim GUI / Glass)**: Full WPILib simulation of robot code and subsystems using `DCMotorSim` and live NetworkTables telemetry.
3. **On-Robot Hardware Verification & Safety Checklist**: Pre-flight procedure and safety guards for running characterization on physical competition mechanisms.

---

## Section 1: Repository Unit & Physics Simulation Suite

The repository contains an automated test suite in [`tests/test_tuning.py`](file:///home/bjarrett/Projects/FROGlib/tests/test_tuning.py) covering mathematical algorithms, state machine transitions, safety limits, and fault injection.

### Running the Test Suite

Run the tests using Python's standard `unittest` runner or `pytest`:

```bash
# Run all tests in the repository
python3 -m unittest discover -s tests -v

# Run the characterization and tuning suite specifically
python3 -m unittest tests/test_tuning.py -v

# Alternatively, using pytest (if installed)
pytest tests/test_tuning.py -v
```

All 19 tests should complete in under 0.1 seconds.

---

### The `SimulatedTalonFXPlant` Physics Model

To validate the characterizer's identification algorithms end-to-end without real hardware, `tests/test_tuning.py` implements a physics plant based on the continuous motor and mechanism differential equation:

$$V_{\text{app}} = k_S \cdot \text{sign}(\omega) + k_V \cdot \omega + k_A \cdot \alpha$$

Where:
- $V_{\text{app}}$: Applied terminal voltage from the TalonFX.
- $\omega$: Rotational velocity ($\text{RPS}$).
- $\alpha = \frac{d\omega}{dt}$: Rotational acceleration ($\text{RPS/s}^2$).
- $k_S$: Static friction / stiction voltage threshold ($\text{V}$).
- $k_V$: Velocity back-EMF constant ($\text{V} / \text{RPS}$).
- $k_A$: Mechanism inertia constant ($\text{V} / (\text{RPS/s}^2)$).

#### Key Simulation Mechanics
- **Static Stiction Regime**: If $\|\omega\| < 10^{-4}\text{ RPS}$ and $\|V_{\text{app}}\| \le k_S$, the plant remains locked ($\omega = 0, \alpha = 0$). Motion only starts once applied voltage breaks away past $k_S$.
- **Dynamic Motion Regime**: When in motion, net accelerating voltage is calculated by subtracting friction and back-EMF:
  $$\alpha = \frac{V_{\text{app}} - \text{copysign}(k_S, \omega) - k_V \cdot \omega}{k_A}$$
- **Numerical Integration**: The plant uses high-resolution sub-stepping ($\le 0.5\text{ ms}$) within each simulation tick to guarantee numerical stability even during rapid acceleration steps.
- **Current Clamping & Winding Resistance**: Stator current is calculated from winding resistance and back-EMF ($I = \frac{\|V - \omega \cdot k_V\|}{R}$), then clamped to the configured `stator_current_limit`.
- **Fault Injection Support**: The plant supports injecting mechanical jams (`plant.jammed = True`) and disabling acceleration signals to verify characterizer resilience.

---

### Test Suite Architecture

The test suite is organized into six targeted test classes:

| Test Class | Focus Area | What It Validates |
| :--- | :--- | :--- |
| **`TestTuningMath`** | Algorithms | Validates ordinary least squares `linear_regression()` on ideal and noisy data sets, handles empty/single-element edge cases, verifies 2nd-order pole placement gain synthesis ($k_P = k_A \omega_n^2$, $k_D = 2 \zeta \omega_n k_A - k_V$), and confirms $k_D$ is clamped to $\ge 0.0$. |
| **`TestTuningResult`** | Data Structures | Confirms `TuningResult` correctly populates Phoenix 6 `Slot0Configs`, formats human-readable text reports, and generates copy-pasteable Python configuration snippets. |
| **`TestGearingAndSafety`** | Gearing & Limits | Tests `validate_gearing()` theoretical sanity check comparing measured $k_V$ against $\frac{12.0}{100.0} \times G$, and verifies that forward limit boundary breaches trigger immediate emergency aborts. |
| **`TestEndToEndPhysicsSimulation`** | State Machine | Runs the complete 9-stage sequence (`KS_FORWARD` $\to$ `RETURN_HOME_1` $\to$ `KS_REVERSE` $\to$ `RETURN_HOME_2` $\to$ `KV_SWEEP` $\to$ `RETURN_HOME_3` $\to$ `KA_STEP` $\to$ `RETURN_HOME_4` $\to$ `COMPUTE_AND_FINISH`) on simulated flywheels and elevators; confirms hot-apply to Slot 0 and Slot 1, and validates fallback $k_A$ estimation when acceleration telemetry is unavailable. |
| **`TestSafetyAndFaultInjection`** | Fault Handling | Injects reverse limit crossings, motor jams (stall current threshold exceeding timeout), and step timeouts; confirms the motor voltage is immediately zeroed and CAN profile reverted. Also tests `FROGSysIdHelper` voltage clamping at soft limits. |
| **`TestFROGTalonFXConvenienceMethods`** | Integration | Verifies that `FROGTalonFX.get_characterizer()` and `FROGTalonFX.get_sysid_helper()` correctly construct and bind helpers with the motor configuration. |

---

### Adding a Custom Simulated Mechanism Test

You can write tests for your robot's custom mechanisms by subclassing `unittest.TestCase` and pairing `SimulatedTalonFXPlant` with `TalonFXCharacterizer`:

```python
import unittest
from unittest.mock import patch
from froglib.ctre import CharacterizationConfig, TalonFXCharacterizer
from tests.test_tuning import SimulatedTalonFXPlant

class TestCustomWristMechanism(unittest.TestCase):
    def test_wrist_characterization(self):
        # 1. Create a simulated plant with known physics constants:
        #    E.g., 25:1 gear reduction, kS=0.35V, kV=3.0V/rps, kA=0.08V/(rps/s)
        plant = SimulatedTalonFXPlant(
            true_ks=0.35,
            true_kv=3.00,
            true_ka=0.08,
            gearing=25.0,
            initial_position=2.0,
        )

        # 2. Configure safety limits for a bounded rotational wrist
        config = CharacterizationConfig(
            mechanism_name="SimWrist",
            is_bounded=True,
            min_test_pos=0.0,
            max_test_pos=8.0,
            safe_home_pos=2.0,
            stator_current_limit_amps=20.0,
            target_bandwidth_rad_s=18.0,
            auto_apply=True,
        )

        characterizer = TalonFXCharacterizer(plant, config)

        # 3. Step through the simulation loop with a virtual clock
        sim_time = 0.0
        dt = 0.005
        with patch("froglib.ctre.time.monotonic", side_effect=lambda: sim_time):
            characterizer.setup()
            for _ in range(3000):  # 15 simulated seconds
                sim_time += dt
                plant.step_physics(dt)
                if characterizer.periodic():
                    break

        # 4. Assertions: routine must complete, recover physics, and return home
        self.assertEqual(characterizer.state, TalonFXCharacterizer.State.COMPUTE_AND_FINISH)
        self.assertAlmostEqual(plant.position, config.safe_home_pos, delta=0.1)

        result = characterizer.get_result()
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.k_s, plant.true_ks, delta=0.03)
        self.assertAlmostEqual(result.k_v, plant.true_kv, delta=0.05)
        self.assertGreater(result.r_squared, 0.99)
```

---

## Section 2: Testing in a Robot Project with WPILib Simulation (Sim GUI / Glass)

Before testing on a physical robot, verify your subsystem and command bindings inside the WPILib desktop simulation environment (Glass or Shuffleboard).

### 1. Install FROGlib in Your Robot Project Environment

In your robot project's virtual environment:

```bash
# Editable install from local FROGlib repository
pip install -e /path/to/FROGlib

# OR install the 2027 alpha pre-release directly from GitHub
pip install git+https://github.com/FROG3160/FROGlib.git@2027.0.0a1
```

### 2. Configure Motor Physics Simulation in Subsystem

Enable physics simulation on `FROGTalonFX` using `simulation_init()` and update the physics plant during `simulationPeriodic()`:

```python
from commands2 import Subsystem
from wpimath.system.plant import DCMotor
from wpilib import RobotController
from froglib.ctre import FROGTalonFX, CharacterizationConfig

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.flywheel_motor = FROGTalonFX(
            id=12,
            signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
            canbus="rio",
        )
        
        # Configure gearing in feedback
        self.flywheel_motor.config.feedback.sensor_to_mechanism_ratio = 1.0

        # Initialize simulation model: single Falcon 500 with moment of inertia 0.001 kg*m^2
        self.flywheel_motor.simulation_init(
            moi=0.001,
            motor_model=DCMotor.falcon500(1),
            gearing=1.0,
        )

    def simulationPeriodic(self):
        """Called automatically every 20ms during WPILib simulation."""
        dt = 0.020
        battery_voltage = RobotController.getBatteryVoltage()
        self.flywheel_motor.simulation_update(dt, battery_voltage)

    def get_characterize_command(self):
        return self.flywheel_motor.create_characterization_command(
            config=CharacterizationConfig(
                mechanism_name="ShooterFlywheel",
                max_voltage=9.0,
                target_bandwidth_rad_s=25.0,
                auto_apply=True,
            )
        )
```

### 3. Expose the Command to the Dashboard in `RobotContainer.py`

```python
# In RobotContainer.py
from wpilib import SmartDashboard

class RobotContainer:
    def __init__(self):
        self.shooter = ShooterSubsystem()

        # Expose the characterization command to SmartDashboard / Glass
        SmartDashboard.putData(
            "Tune Shooter Flywheel",
            self.shooter.get_characterize_command(),
        )
```

### 4. Running the Simulation and Inspecting Telemetry

1. Start WPILib simulation:
   ```bash
   robotpy sim
   # or
   python3 robot.py sim
   ```
2. In the WPILib Sim GUI, enable **Test Mode** or **Teleop Mode**.
3. Open **Glass** (`WPILib -> NetworkTables -> Glass`).
4. Find the characterization button under `SmartDashboard -> Tune Shooter Flywheel` and click it.
5. Watch live telemetry published to NetworkTables under `FROGLib/Tuning/ShooterFlywheel`:
   - `State`: Transitions across states:
     - `KS_FORWARD` $\to$ `RETURN_HOME_1`
     - `KS_REVERSE` $\to$ `RETURN_HOME_2`
     - `KV_SWEEP` $\to$ `RETURN_HOME_3`
     - `KA_STEP` $\to$ `RETURN_HOME_4`
     - `COMPUTE_AND_FINISH`
   - `AppliedVolts`: Ramps slowly up to breakaway, steps through steady-state voltages, then pulses acceleration voltage.
   - `VelocityRPS`: Real-time rotational velocity response.
6. Check the terminal console for the printed characterization report and confirmation of gain application.

---

## Section 3: On-Robot Hardware Verification & Safety Checklist

Follow this checklist before running automated motor tuning on a physical robot.

### Pre-Flight Safety Checklist

- [ ] **1. Mechanical Clearance**: Ensure mechanism travel is free of tools, wires, and human hands.
- [ ] **2. Inversion Direction**: Verify that applying positive voltage drives the mechanism in the positive position/velocity direction. If inverted, set `config.motor_output.inverted` before tuning.
- [ ] **3. Feedback Gearing (`sensor_to_mechanism_ratio`)**:
  - Verify that `motor.config.feedback.sensor_to_mechanism_ratio` is set to the true physical ratio (e.g. 10.0 for a 10:1 gear reduction).
  - Confirm that 1 mechanism rotation matches the expected real-world rotation or linear travel.
- [ ] **4. Travel Boundaries for Limited Mechanisms**:
  - For elevators, arms, and wrists, **ALWAYS** set `is_bounded=True`.
  - Set `min_test_pos` safely above the lower physical hard stop.
  - Set `max_test_pos` safely below the upper physical hard stop.
  - Set `safe_home_pos` to a safe equilibrium position.
  - Verify `soft_limit_buffer` (default 0.5 mechanism rotations).
- [ ] **5. Conservative Stator Current Limit**:
  - Set `stator_current_limit_amps` to 15.0A–20.0A during characterization. If the mechanism encounters resistance or hard stops, the motor will stall without damaging gears or tripping breakers.
- [ ] **6. Emergency Stop Ready**:
  - Have a team member hold the spacebar (E-stop in Driver Station) or keep a hand on the physical robot circuit breaker.

---

### Evaluating Characterization Results

Upon completion, `TalonFXCharacterizer` prints the analysis report:

```text
============================================================
  CHARACTERIZATION REPORT: Elevator
============================================================
Coordinate Frame: Mechanism Units (Gearing = 12.50:1)
R² Fit Score    : 0.9988 (Optimal)
kS (Static)     : 0.2850 V
kV (Velocity)   : 1.5020 V / rps
kA (Accel)      : 0.0380 V / (rps/s)
Recommended kP  : 24.3200 V / rotation
Recommended kD  : 0.7380 V / rps
============================================================
```

#### Diagnostic Criteria:

| Metric | Target | Diagnosis if Outside Target |
| :--- | :--- | :--- |
| **$R^2$ Fit Score** | $\ge 0.98$ | If $R^2 < 0.98$, the data is noisy or non-linear. Check for mechanical binding, excessive backlash, loose belt tension, or electrical noise. Increase `kv_settle_time_s`. |
| **Gearing Validation** | Deviation $< 40\%$ | The characterizer compares measured $k_V$ against theoretical free speed: $\frac{12.0}{100.0} \times G$. If a warning triggers, verify `sensor_to_mechanism_ratio`. |
| **$k_S$ Value** | $0.1\text{V} - 0.6\text{V}$ | Unusually high $k_S$ ($> 1.0\text{V}$) indicates excessive mechanical friction or misaligned bearings. |
| **$k_A$ Value** | Positive ($> 0.0$) | If $k_A \le 0.0$, the step test duration was too short or acceleration telemetry was noisy. |

---

### Fallback to Stable Release Track

If you encounter unexpected hardware or software issues in the field during a competition event, you can instantly fall back to the **`2026.1.2`** stable release track without modifying your core robot subsystem logic:

```bash
# Switch environment to 2026 stable release
pip install --force-reinstall git+https://github.com/FROG3160/FROGlib.git@2026.1.2
```

In your robot code, revert from `create_characterization_command()` to manual gain configuration via standard `Slot0Configs`:

```python
from phoenix6.configs import Slot0Configs

slot0 = Slot0Configs()
slot0.k_s = 0.25
slot0.k_v = 0.12
slot0.k_p = 4.5
slot0.k_d = 0.15
motor.configurator.apply(slot0)
```
