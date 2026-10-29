# TalonFX Gain Tuning & Characterization Guide

This guide explains how to determine firmware gains for CTRE **TalonFX** motor controllers running Phoenix 6 in **FROGlib**.

FROGlib supports two workflows:
1. **Autonomous On-Robot Tuning (`TalonFXCharacterizer`)**: Runs directly on the robot (or in simulation) to test $k_S, k_V, k_A$, synthesize $k_P, k_D$, and output/apply `Slot0Configs` immediately without manual log export.
2. **WPILib SysIdRoutine Integration (`FROGSysIdHelper`)**: Automates WPILib's `SysIdRoutine` with CTRE Phoenix 6's high-speed `SignalLogger` (250 Hz frame rates, `.hoot` logging) for deep offline graphical analysis in the SysId desktop application.

---

## 1. Phoenix 6 Gain Architecture & Units

In CTRE Phoenix 6, closed-loop controllers run on-board the TalonFX at **1 kHz**. The controller uses feedforward and feedback gains configured in `Slot0Configs` (or slots 1–2):

| Gain | Name | Units (Voltage Mode) | Description |
| :--- | :--- | :--- | :--- |
| **$k_S$** | Static Friction | $\text{Volts}$ | Voltage required to overcome static friction / stiction so the motor initiates motion. |
| **$k_V$** | Velocity Feedforward | $\text{Volts} / \text{rps}$ | Voltage required per unit of velocity to counter back-EMF ($k_V \approx \frac{12\text{ V}}{\text{Free Speed in RPS}}$). |
| **$k_A$** | Acceleration Feedforward | $\text{Volts} / (\text{rps/s})$ | Voltage required per unit of acceleration to overcome mechanism inertia ($J$). |
| **$k_G$** | Gravity Feedforward | $\text{Volts}$ | Constant voltage for vertical lifts (`Elevator_Static`) or $V / \cos(\theta)$ for arms (`Arm_Cosine`). |
| **$k_P$** | Proportional Feedback | $\text{V / rot}$ (pos) or $\text{V / rps}$ (vel) | Error-correcting voltage per unit of error. Calculated via continuous 2nd-order pole placement ($k_P = k_A \cdot \omega_n^2$). |
| **$k_D$** | Derivative Feedback | $\text{V / rps}$ (pos) or $\text{V} / (\text{rps/s})$ (vel) | Damping voltage per rate of change of error ($k_D = 2 \zeta \omega_n k_A - k_V$). |

---

## 2. Gearing & Mechanism Units

### Recommended: Configure Gearing in Firmware
Set `sensor_to_mechanism_ratio` in the motor's feedback configuration:

```python
config.feedback.sensor_to_mechanism_ratio = 10.0  # 10 rotor turns = 1 mechanism turn
```

When configured:
* All signals (`get_position()`, `get_velocity()`, `get_acceleration()`) return **mechanism rotations**, **mechanism RPS**, and **mechanism $\text{RPS/s}^2$** directly.
* `TalonFXCharacterizer` automatically computes gains in **mechanism units**.
* The resulting `Slot0Configs` can be applied directly to the motor with zero manual conversion!

### Physics-Based Gearing Validation
The characterizer automatically cross-checks your measured $k_V$ against theoretical free speed:
$$\text{Theoretical } k_V \approx \frac{12.0\text{ V}}{100\text{ RPS}} \times G$$
If measured $k_V$ deviates by $> 40\%$ from theoretical expectation, the characterizer flags a warning that `sensor_to_mechanism_ratio` may be missing or inverted.

---

## 3. Safety for Limited-Travel Mechanisms (Arms, Elevators)

To prevent mechanisms from slamming into hard stops, stripping gears, or burning motor windings, FROGlib implements a multi-layer defense:

1. **Firmware Soft Limits (`SoftwareLimitSwitchConfigs`)**: Enforced on-board the TalonFX at **1 kHz**. The controller cuts voltage instantly if a limit is reached.
2. **Current Clamping (`stator_current_limit_amps`)**: Normal operation may use 40A–80A, but during testing, current is clamped to a safe threshold (e.g., 15A–20A).
3. **Virtual Test Envelope (`min_test_pos`, `max_test_pos`, `soft_limit_buffer`)**: Testing is restricted to a safe middle window (e.g., 60–70% of total travel).
4. **Dynamic Stall Detection**: Monitors stator current and velocity. If the motor binds or stalls, an emergency abort is triggered immediately.
5. **Auto-Return to Home**: The routine automatically drives back to `safe_home_pos` between test stages.

---

## 4. Workflow 1: Autonomous On-Robot Characterizer

### Example: Continuous Mechanism (Shooter Flywheel)

```python
from commands2 import Subsystem
from froglib.ctre import FROGTalonFX, CharacterizationConfig

class FlywheelSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=15,
            signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
            canbus="canivore",
        )

    def get_characterization_command(self):
        """Returns the Command to run during testing."""
        return self.motor.create_characterization_command(
            config=CharacterizationConfig(
                mechanism_name="ShooterFlywheel",
                max_voltage=10.0,
                target_bandwidth_rad_s=25.0,  # omega_n responsiveness
                auto_apply=True,              # Hot-apply gains directly to Slot 0
            )
        )
```

### Example: Bounded Mechanism (Elevator / Arm)

```python
from commands2 import Subsystem
from froglib.ctre import FROGTalonFX, CharacterizationConfig

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=20,
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
            canbus="rio",
        )

    def get_characterization_command(self):
        return self.motor.create_characterization_command(
            config=CharacterizationConfig(
                mechanism_name="Elevator",
                is_bounded=True,
                min_test_pos=1.0,              # Rotations above bottom hard stop
                max_test_pos=18.0,             # Rotations below top hard stop
                safe_home_pos=5.0,             # Reset position between tests
                soft_limit_buffer=0.5,         # Keep-out buffer
                stator_current_limit_amps=18.0, # Safe current cap during test
                ramp_rate_v_per_s=1.0,         # Faster ramp so breakaway occurs quickly
                step_timeout_s=1.5,
                target_bandwidth_rad_s=20.0,
            )
        )
```

### Binding the Command in `RobotContainer.py`

Bind the command to a button chord or dashboard trigger in `testInit`:

```python
# In RobotContainer.py
from wpilib import SmartDashboard

SmartDashboard.putData(
    "Tune Flywheel",
    self.flywheel.get_characterization_command()
)
```

### Interpreting the Results
Upon completion, the characterizer outputs a full report to the console and NetworkTables:

```text
============================================================
  CHARACTERIZATION REPORT: ShooterFlywheel
============================================================
Coordinate Frame: Mechanism Units (Gearing = 1.00:1)
R² Fit Score    : 0.9984 (Optimal)
kS (Static)     : 0.2350 V
kV (Velocity)   : 0.1185 V / rps
kA (Accel)      : 0.0076 V / (rps/s)
Recommended kP  : 4.7500 V / rotation
Recommended kD  : 0.1845 V / rps
============================================================
```

---

## 5. Workflow 2: WPILib SysIdRoutine with SignalLogger

If you prefer using the desktop **WPILib SysId** application to visually inspect raw data curves and perform state-space system identification:

```python
from froglib.ctre import FROGTalonFX

class ArmSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(id=25)

        self.sysid_helper = self.motor.get_sysid_helper(
            mechanism_name="Arm",
            ramp_rate_v_per_s=0.5,
            step_voltage=4.0,
            is_bounded=True,
            min_pos=0.0,
            max_pos=15.0,
        )

    # Expose the four SysId commands
    def sysid_quasistatic_forward(self):
        return self.sysid_helper.quasistatic_forward()

    def sysid_quasistatic_reverse(self):
        return self.sysid_helper.quasistatic_reverse()

    def sysid_dynamic_forward(self):
        return self.sysid_helper.dynamic_forward()

    def sysid_dynamic_reverse(self):
        return self.sysid_helper.dynamic_reverse()
```

### Procedure:
1. Run the 4 routines (`quasistatic_forward`, `quasistatic_reverse`, `dynamic_forward`, `dynamic_reverse`) via your dashboard.
2. `SignalLogger` automatically records high-frequency telemetry at 250 Hz into a `.hoot` file on the roboRIO.
3. Open **Phoenix Tuner X**, go to **Tools -> Log Extractor**, and convert the `.hoot` file to `.wpilog`.
4. Open the **SysId** desktop application, load the `.wpilog`, and run the analysis.
